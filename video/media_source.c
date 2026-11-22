/*
 * media_source.c 把编码器传来的一帧 H.264 分发给当前的所有订阅者。
 * video_encoder.c 每帧推送一次。RTSP 的 RTP 发送器、常规 MP4 录像和
 * DMS 事件录像各自订阅，分别在自己的线程里处理同一路码流。
 *
 * 每个订阅者都有独立的队列。处理慢时只积压自己的队列，队列满时丢最老的一帧。
 * 编码器负责复制入队，发送网络数据或写 MP4 的操作在订阅回调中执行。
 */

#include "media_source.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 每个订阅者最多等待 8 帧，避免网络慢或写盘慢时无限积压。 */
#define MEDIA_QUEUE_DEPTH 8

/*
 * queue 是环形队列。head 指向下一帧要取的位置，count 是等待的帧数。
 * 入队位置是 (head + count) % MEDIA_QUEUE_DEPTH，取出后推进 head。
 * lock 保护队列和运行状态，cond 用来唤醒等待中的订阅线程。
 */
typedef struct media_subscriber {
    struct media_subscriber *next;
    media_frame_cb cb;
    void *arg;
    pthread_t thread;
    pthread_mutex_t lock;
    pthread_cond_t cond;
    media_frame queue[MEDIA_QUEUE_DEPTH];
    td_u32 head;
    td_u32 count;
    td_bool running;
    td_u64 dropped;
} media_subscriber;

/* g_source_lock 保护订阅者链表和全局计数。投递时先拿它，再拿队列锁。 */
static media_subscriber *g_subscribers = NULL;
static pthread_mutex_t g_source_lock = PTHREAD_MUTEX_INITIALIZER;
static td_u64 g_pushed = 0;
static td_u64 g_dropped = 0;

static void *media_subscriber_thread(void *arg)
{
    media_subscriber *sub = (media_subscriber *)arg;

    while (1) {
        pthread_mutex_lock(&sub->lock);

        /* 条件等待会释放队列锁，醒来后重新持锁并检查队列。 */
        while (sub->count == 0 && sub->running) {
            pthread_cond_wait(&sub->cond, &sub->lock);
        }

        /* 取消订阅后把已入队的帧处理完，再退出。 */
        if (!sub->running && sub->count == 0) {
            pthread_mutex_unlock(&sub->lock);
            break;
        }

        /*
         * 把帧描述从队列拷到局部变量。这里只复制指针和元数据，
         * 码流内存由本线程接管，队列槽马上可以放下一帧。
         */
        media_frame frame = sub->queue[sub->head];
        sub->head = (sub->head + 1) % MEDIA_QUEUE_DEPTH;
        sub->count--;
        pthread_mutex_unlock(&sub->lock);

        /* 先解锁再调用回调，让生产者可以在回调执行时继续入队。 */
        if (sub->cb) {
            (void)sub->cb(&frame, sub->arg);
        }
        free(frame.data);
    }

    return NULL;
}

void *media_source_subscribe(media_frame_cb cb, void *arg)
{
    media_subscriber *sub = NULL;
    td_s32 ret;

    if (cb == NULL) {
        printf("Failed to subscribe media consumer: callback is NULL\n");
        return NULL;
    }

    sub = (media_subscriber *)calloc(1, sizeof(media_subscriber));
    if (!sub) {
        printf("Failed to subscribe media consumer: malloc failed\n");
        return NULL;
    }

    sub->cb = cb;
    sub->arg = arg;
    sub->head = 0;
    sub->count = 0;
    sub->running = TD_TRUE;
    sub->dropped = 0;

    ret = pthread_mutex_init(&sub->lock, NULL);
    if (ret != 0) {
        printf("Failed to subscribe media consumer: pthread_mutex_init failed\n");
        goto free_sub;
    }
    ret = pthread_cond_init(&sub->cond, NULL);
    if (ret != 0) {
        printf("Failed to subscribe media consumer: pthread_cond_init failed\n");
        goto destroy_lock;
    }

    /* 队列、锁和线程都准备好后，再把订阅者公布给取流线程。 */
    ret = pthread_create(&sub->thread, NULL, media_subscriber_thread, sub);
    if (ret != 0) {
        printf("Failed to subscribe media consumer: pthread_create failed\n");
        goto destroy_cond;
    }

    pthread_mutex_lock(&g_source_lock);
    sub->next = g_subscribers;
    g_subscribers = sub;
    pthread_mutex_unlock(&g_source_lock);

    return sub;

destroy_cond:
    pthread_cond_destroy(&sub->cond);
destroy_lock:
    pthread_mutex_destroy(&sub->lock);
free_sub:
    free(sub);
    return NULL;
}

void media_source_unsubscribe(void *handle)
{
    media_subscriber *sub = (media_subscriber *)handle;
    media_subscriber **link = &g_subscribers;

    if (sub == NULL) {
        return;
    }

    /*
     * 先从链表移除，之后就不会再收到新帧。link 指向链表头或上一节点的
     * next 字段，因此删除头节点和删除中间节点可以使用同一条赋值语句。
     */
    pthread_mutex_lock(&g_source_lock);
    while (*link && *link != sub) {
        link = &(*link)->next;
    }
    if (*link == sub) {
        *link = sub->next;
    }
    pthread_mutex_unlock(&g_source_lock);

    /* 唤醒线程并等它处理完队列。join 返回前，回调仍可能访问 arg。 */
    pthread_mutex_lock(&sub->lock);
    sub->running = TD_FALSE;
    pthread_cond_signal(&sub->cond);
    pthread_mutex_unlock(&sub->lock);
    pthread_join(sub->thread, NULL);

    while (sub->count > 0) {
        free(sub->queue[sub->head].data);
        sub->head = (sub->head + 1) % MEDIA_QUEUE_DEPTH;
        sub->count--;
    }
    pthread_mutex_destroy(&sub->lock);
    pthread_cond_destroy(&sub->cond);
    free(sub);
}

td_s32 media_source_push(const td_u8 *data, td_u32 len, td_u64 pts, td_bool is_frame_end)
{
    media_subscriber *sub;

    if (data == NULL || len == 0) {
        printf("Failed to push frame: invalid argument\n");
        return TD_FAILURE;
    }
    if (len > MEDIA_FRAME_MAX_SIZE) {
        printf("Failed to push frame: length %u exceeds limit\n", len);
        return TD_FAILURE;
    }

    /* 遍历期间持有链表锁，订阅和取消订阅会在本次遍历结束后进行。 */
    pthread_mutex_lock(&g_source_lock);
    g_pushed++;
    for (sub = g_subscribers; sub != NULL; sub = sub->next) {
        pthread_mutex_lock(&sub->lock);

        /* 丢掉最早的待处理帧，给最新帧腾出一格。 */
        if (sub->count == MEDIA_QUEUE_DEPTH) {
            free(sub->queue[sub->head].data);
            sub->head = (sub->head + 1) % MEDIA_QUEUE_DEPTH;
            sub->count--;
            sub->dropped++;
            g_dropped++;
        }

        /* 每个订阅者保存一份独立的码流副本，回调结束后各自释放。 */
        media_frame *queued_frame = &sub->queue[(sub->head + sub->count) % MEDIA_QUEUE_DEPTH];
        queued_frame->data = (td_u8 *)malloc(len);
        if (!queued_frame->data) {
            printf("Failed to push frame: malloc failed\n");
            pthread_mutex_unlock(&sub->lock);
            continue;
        }
        memcpy(queued_frame->data, data, len);
        queued_frame->len = len;
        queued_frame->pts = pts;
        queued_frame->is_frame_end = is_frame_end;
        sub->count++;

        pthread_cond_signal(&sub->cond);
        pthread_mutex_unlock(&sub->lock);
    }
    pthread_mutex_unlock(&g_source_lock);

    return TD_SUCCESS;
}

void media_source_get_stats(td_u64 *pushed, td_u64 *dropped)
{
    pthread_mutex_lock(&g_source_lock);
    if (pushed != NULL) {
        *pushed = g_pushed;
    }
    if (dropped != NULL) {
        *dropped = g_dropped;
    }
    pthread_mutex_unlock(&g_source_lock);
}
