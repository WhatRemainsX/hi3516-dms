/*
 * 事件录像在 APP_DMS_CLIP_PATH 所在目录生成 dms_event_*.mp4。
 *
 *   media_source → dms_clip_on_frame()     提供每一帧 H.264，平时也缓存
 *   DMS 报警事件 → dms_clip_request()      通知何时开始或延长录像
 *
 * 这两个输入汇入同一个片段。订阅线程处理请求，根据缓存中的 IDR
 * 关键帧开始写 MP4，然后把后续画面接到同一文件。停止时先取消订阅，
 * 等待回调退出后，再封好尚未结束的文件。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <time.h>
#include <unistd.h>

#include "app_config.h"
#include "dms_clip.h"

#include "media_source.h"
#include "mp4_writer.h"
#include "nal_parser.h"

#if APP_DMS_CLIP_ENABLED

/*
 * 订阅是常驻的。每来一帧就复制到预分配字节池，因为 media_source 的
 * data 在回调返回后会释放。实际回写长度由可用的起始关键帧决定。
 *
 * 槽位数按 30fps × (pre + 2 秒 GOP 余量) 算，默认 (10+2)*30 = 360。
 * 所有帧数据共享一块 APP_DMS_CLIP_RING_MAX_BYTES 的字节池，运行期不再
 * malloc/free，避免 32MB 板上长期运行时的堆碎片。
 */
#define DMS_CLIP_RING_SLOTS ((APP_DMS_CLIP_PRE_SECONDS + 2) * APP_VIDEO_FPS)

typedef struct {
    td_u32  offset;  /* 在 g_ring_pool 内的偏移 */
    td_u32  len;
    td_u64  pts;     /* 微秒，与 media_frame.pts 同一时间基 */
    td_bool is_idr;  /* 这一帧是否含 IDR 切片（入队时用 nal_parse_frame 判断） */
} dms_clip_slot;

static void *g_clip_subscription = NULL;
static mp4_writer *g_clip_writer = NULL;
static pthread_mutex_t g_clip_lock = PTHREAD_MUTEX_INITIALIZER;
static atomic_bool g_clip_requested = ATOMIC_VAR_INIT(false);

static dms_clip_slot g_ring[DMS_CLIP_RING_SLOTS];
static td_u8 g_ring_pool[APP_DMS_CLIP_RING_MAX_BYTES];
static td_u32 g_ring_head = 0;    /* 最老帧所在槽位下标 */
static td_u32 g_ring_count = 0;   /* 环内帧数 */
static td_u64 g_ring_bytes = 0;   /* 环内拷贝占用的总字节数 */
static td_u32 g_ring_tail = 0;    /* 下一帧追加位置（池内偏移） */

static td_u64 g_clip_stop_pts_us = 0;
/* 回退模式：找不到可回写的 IDR 时，等下一个 IDR 再开文件（原行为）。 */
static td_bool g_clip_wait_idr = TD_FALSE;
static td_bool g_clip_request_once = TD_FALSE;
static double g_clip_last_request_ms = 0.0;
static double g_clip_last_cooldown_log_ms = -1.0;

static double dms_clip_now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1000000.0;
}

void dms_clip_request(void)
{
    /*
     * 只置标志：环形缓冲和 mp4_writer 都在订阅回调线程里操作，
     * 这里不能直接开文件写盘。回调里取"当前帧 pts"当触发时刻。
     * 注意冷却只看“空闲且未等 IDR”的请求；录像中收到的请求会保留为延长信号。
     */
    double now = dms_clip_now_ms();
    double cooldown_ms = (double)APP_DMS_CLIP_COOLDOWN_SECONDS * 1000.0;

    /*
     * 下面读写的 g_clip_writer / g_clip_wait_idr / g_clip_request_once /
     * g_clip_last_request_ms 都归订阅回调线程所有（在 g_clip_lock 下修改），
     * 这里在 DMS 线程里也必须持锁访问。用 trylock：回调正在回写预录时不让
     * DMS 线程卡住；锁忙说明回调正在跑，此时只置请求标志，交给回调处理。
     */
    if (pthread_mutex_trylock(&g_clip_lock) != 0) {
        atomic_store(&g_clip_requested, TD_TRUE);
        return;
    }

    /*
     * 冷却只拦“空闲且不在等 IDR”的新录像。录像中或等 IDR 时的请求必须放行，
     * 否则录像窗口内的新事件到不了 dms_clip_on_frame，延长停止时间的分支永远是死代码。
     */
    if (g_clip_writer == NULL && g_clip_wait_idr == TD_FALSE &&
        g_clip_request_once == TD_TRUE && now - g_clip_last_request_ms < cooldown_ms) {
        if (g_clip_last_cooldown_log_ms < 0.0 ||
            now - g_clip_last_cooldown_log_ms >= cooldown_ms) {
            printf("dms_clip: request suppressed by %ds cooldown\n",
                   APP_DMS_CLIP_COOLDOWN_SECONDS);
            g_clip_last_cooldown_log_ms = now;
        }
        pthread_mutex_unlock(&g_clip_lock);
        return;
    }
    g_clip_request_once = TD_TRUE;
    /* 这里只置请求，不开始计时；真正开始计冷却的时刻是录像结束或失败放弃时。 */
    atomic_store(&g_clip_requested, TD_TRUE);
    pthread_mutex_unlock(&g_clip_lock);
}

/* 丢掉环里最老的一帧；字节池内容由后续追加自然覆盖。 */
static void dms_clip_ring_drop_oldest(void)
{
    if (g_ring_count == 0) {
        return;
    }
    if (g_ring_bytes >= g_ring[g_ring_head].len) {
        g_ring_bytes -= g_ring[g_ring_head].len;
    } else {
        g_ring_bytes = 0;
    }
    g_ring_head = (g_ring_head + 1) % DMS_CLIP_RING_SLOTS;
    g_ring_count--;
    if (g_ring_count == 0) {
        g_ring_head = 0;
        g_ring_tail = 0;
        g_ring_bytes = 0;
    }
}

static void dms_clip_ring_reset(void)
{
    while (g_ring_count > 0) {
        dms_clip_ring_drop_oldest();
    }
    g_ring_bytes = 0;
    g_ring_head = 0;
    g_ring_tail = 0;
}

/*
 * 字节池按追加方式使用。尾部空间不够时，把当前有效帧整体搬回池首；
 * 这是低频压缩（约每个环容量周期一次），不产生额外堆分配。
 */
static void dms_clip_ring_compact(void)
{
    td_u32 rel;
    td_u32 idx;
    td_u32 oldest_offset;

    if (g_ring_count == 0) {
        g_ring_tail = 0;
        return;
    }
    oldest_offset = g_ring[g_ring_head].offset;
    if (oldest_offset == 0) {
        return;
    }

    memmove(g_ring_pool, g_ring_pool + oldest_offset, (size_t)g_ring_bytes);
    for (rel = 0; rel < g_ring_count; rel++) {
        idx = (g_ring_head + rel) % DMS_CLIP_RING_SLOTS;
        g_ring[idx].offset -= oldest_offset;
    }
    g_ring_tail -= oldest_offset;
}

/* 这一帧里有没有 IDR 切片（NAL type 5）。 */
static td_bool dms_clip_frame_is_idr(const media_frame *frame)
{
    nal_frame_info info;
    td_u32 i;

    if (nal_parse_frame(frame->data, frame->len, &info) != TD_SUCCESS) {
        return TD_FALSE;
    }
    for (i = 0; i < info.count; i++) {
        if (info.nals[i].type == 5) {   /* NAL_TYPE_IDR */
            return TD_TRUE;
        }
    }
    return TD_FALSE;
}

/* 把一帧拷进预分配字节池，并严格按 APP_DMS_CLIP_RING_MAX_BYTES 裁剪。 */
static void dms_clip_ring_push(const media_frame *frame)
{
    td_u32 idx;

    if (frame->data == NULL || frame->len == 0 || frame->len > MEDIA_FRAME_MAX_SIZE) {
        return;
    }
    if (frame->len > APP_DMS_CLIP_RING_MAX_BYTES) {
        printf("dms_clip: frame too large for ring (%u bytes), dropped\n", frame->len);
        return;
    }

    /* 槽位满了先丢一帧。 */
    if (g_ring_count >= DMS_CLIP_RING_SLOTS) {
        dms_clip_ring_drop_oldest();
    }

    /* 字节池到达上限时丢最老帧，保证运行期内存占用有硬上限。 */
    while (g_ring_count > 0 &&
           g_ring_bytes + (td_u64)frame->len > (td_u64)APP_DMS_CLIP_RING_MAX_BYTES) {
        dms_clip_ring_drop_oldest();
    }
    if (g_ring_count == 0) {
        g_ring_head = 0;
        g_ring_tail = 0;
        g_ring_bytes = 0;
    }
    if ((td_u64)g_ring_tail + (td_u64)frame->len >
        (td_u64)APP_DMS_CLIP_RING_MAX_BYTES) {
        dms_clip_ring_compact();
    }

    idx = (g_ring_head + g_ring_count) % DMS_CLIP_RING_SLOTS;
    memcpy(g_ring_pool + g_ring_tail, frame->data, frame->len);
    g_ring[idx].offset = g_ring_tail;
    g_ring[idx].len = frame->len;
    g_ring[idx].pts = frame->pts;
    g_ring[idx].is_idr = dms_clip_frame_is_idr(frame);
    g_ring_tail += frame->len;
    g_ring_count++;
    g_ring_bytes += frame->len;
}

/*
 * 从预录缓冲选起点，取触发时刻当时或之前的最近一个 IDR，查找范围为
 * pre + 2 秒。额外 2 秒用于覆盖关键帧间隔。
 * 返回环内相对位置。DMS_CLIP_RING_SLOTS 表示没找到合格的关键帧。
 */
static td_u32 dms_clip_ring_find_start(td_u64 trigger_pts_us)
{
    td_u64 pre_us = (td_u64)APP_DMS_CLIP_PRE_SECONDS * 1000000ULL;
    td_u64 min_pts;
    td_u32 rel;
    td_u32 best = DMS_CLIP_RING_SLOTS;

    if (trigger_pts_us > pre_us + 2000000ULL) {
        min_pts = trigger_pts_us - pre_us - 2000000ULL;
    } else {
        min_pts = 0;
    }

    for (rel = 0; rel < g_ring_count; rel++) {
        td_u32 idx = (g_ring_head + rel) % DMS_CLIP_RING_SLOTS;
        if (g_ring[idx].is_idr == TD_FALSE) {
            continue;
        }
        if (g_ring[idx].pts > trigger_pts_us || g_ring[idx].pts < min_pts) {
            continue;
        }
        best = rel;   /* 升序遍历，最后赋值的就是最新的合格 IDR */
    }
    return best;
}

/*
 * 生成事件文件名：dms_event_YYYYMMDD_HHMMSS.mp4。
 * 目录沿用 APP_DMS_CLIP_PATH 所在的目录（那个宏现在只用来定目录，不再当文件名）。
 * 时间戳取板端时钟（UTC）：板子没有 RTC/NTP 时可能是 1970；同一秒内多次触发用
 * "_NN" 后缀区分，避免互相覆盖。若以后想完全摆脱板端时钟，可改成开机以来的单调序号。
 */
/*
 * 把 time_t（UTC 秒）拆成 年/月/日 时:分:秒。
 * 自己算而不用 localtime_r：一是板端可能没配时区/TZ，二是 MinGW 主机默认不声明
 * localtime_r（要 _POSIX_C_SOURCE），自算两边都能编、结果还是确定的 UTC。
 */
static void dms_clip_utc_time(time_t t, int *year, int *mon, int *day,
                              int *hour, int *min, int *sec)
{
    long long days;
    long long secs;
    long long z, era, doe, yoe, y, doy, mp, d, m;

    if (t < 0) {
        t = 0;   /* 1970 之前的一律按 1970 算 */
    }
    days = (long long)t / 86400;
    secs = (long long)t % 86400;

    /* 把"距 1970-01-01 的天数"换算成公历日期（公认的 civil-from-days 算法）。 */
    z = days + 719468;
    era = (z >= 0 ? z : z - 146096) / 146097;
    doe = z - era * 146097;                                         /* [0, 146096] */
    yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;   /* [0, 399] */
    y = yoe + era * 400;
    doy = doe - (365 * yoe + yoe / 4 - yoe / 100);                 /* [0, 365] */
    mp = (5 * doy + 2) / 153;                                       /* [0, 11] */
    d = doy - (153 * mp + 2) / 5 + 1;                               /* [1, 31] */
    m = mp + (mp < 10 ? 3 : -9);                                    /* [1, 12] */
    y += (m <= 2) ? 1 : 0;

    *year = (int)y;
    *mon = (int)m;
    *day = (int)d;
    *hour = (int)(secs / 3600);
    *min = (int)((secs / 60) % 60);
    *sec = (int)(secs % 60);
}

static void dms_clip_make_path(char *out, size_t out_size)
{
    const char *slash = strrchr(APP_DMS_CLIP_PATH, '/');
    char dir[192];
    time_t now = time(NULL);
    int year, mon, day, hour, min, sec;
    int seq;

    if (slash != NULL) {
        size_t dlen = (size_t)(slash - APP_DMS_CLIP_PATH);
        if (dlen == 0) {
            dlen = 1;   /* "/xxx.mp4"：目录就是根 "/" */
        }
        if (dlen >= sizeof(dir)) {
            dlen = sizeof(dir) - 1;
        }
        memcpy(dir, APP_DMS_CLIP_PATH, dlen);
        dir[dlen] = '\0';
    } else {
        snprintf(dir, sizeof(dir), ".");
    }

    dms_clip_utc_time(now, &year, &mon, &day, &hour, &min, &sec);

    for (seq = 0; seq < 99; seq++) {
        if (seq == 0) {
            snprintf(out, out_size, "%s/dms_event_%04d%02d%02d_%02d%02d_%02d.mp4",
                     dir, year, mon, day, hour, min, sec);
        } else {
            snprintf(out, out_size, "%s/dms_event_%04d%02d%02d_%02d%02d_%02d_%02d.mp4",
                     dir, year, mon, day, hour, min, sec, seq);
        }
        if (access(out, F_OK) != 0) {
            break;   /* 这个名字的文件还不存在，用它 */
        }
    }
}

/*
 * 触发时开文件：从环里找到的 IDR 起，按 pts 顺序把缓冲帧写入，再继续写实时帧。
 * trigger_pts_us 取回调当前帧的 pts（请求到达后下一帧就处理，误差 <= 1 帧周期）。
 */
static td_s32 dms_clip_begin_from_ring(const media_frame *frame)
{
    td_u8 sps[NAL_SPS_MAX_SIZE];
    td_u8 pps[NAL_PPS_MAX_SIZE];
    td_u32 sps_len = 0;
    td_u32 pps_len = 0;
    char path[256];
    td_u32 start_rel;
    td_u32 rel;
    td_u32 idx;
    td_s32 ret;

    start_rel = dms_clip_ring_find_start(frame->pts);
    if (start_rel == DMS_CLIP_RING_SLOTS) {
        return TD_FAILURE;   /* 调用方转回退模式 */
    }

    /* MP4 解码器必须先拿到 SPS/PPS，缺了它后面的 IDR 也解不出来。 */
    ret = nal_get_sps_pps(sps, &sps_len, pps, &pps_len);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }

    dms_clip_make_path(path, sizeof(path));
    g_clip_writer = mp4_open(path, APP_VPSS_OUT_WIDTH, APP_VPSS_OUT_HEIGHT,
                             sps, sps_len, pps, pps_len);
    if (g_clip_writer == NULL) {
        printf("dms_clip: open %s failed\n", path);
        return TD_FAILURE;
    }

    g_clip_stop_pts_us = frame->pts + (td_u64)APP_DMS_CLIP_POST_SECONDS * 1000000ULL;

    /* 从找到的 IDR 开始按 pts 顺序回写；只写 IDR 之后的数据，早期 P 帧没有参考图。 */
    for (rel = start_rel; rel < g_ring_count; rel++) {
        idx = (g_ring_head + rel) % DMS_CLIP_RING_SLOTS;
        ret = mp4_write_frame(g_clip_writer, g_ring_pool + g_ring[idx].offset,
                              g_ring[idx].len, g_ring[idx].pts);
        if (ret != TD_SUCCESS) {
            printf("dms_clip: write pre-roll frame failed, closing %s\n", path);
            (void)mp4_close(g_clip_writer);
            g_clip_writer = NULL;
            return TD_FAILURE;
        }
    }

    idx = (g_ring_head + start_rel) % DMS_CLIP_RING_SLOTS;
    printf("dms_clip: recording started -> %s (pre-roll %u frame(s) from IDR pts=%llu)\n",
           path, g_ring_count - start_rel, (unsigned long long)g_ring[idx].pts);
    return TD_SUCCESS;
}

/* 回退模式：等下一个 IDR，直接从这一帧开文件（与原来的行为一致，没有 pre-roll）。 */
static td_s32 dms_clip_begin_fallback(const media_frame *frame)
{
    td_u8 sps[NAL_SPS_MAX_SIZE];
    td_u8 pps[NAL_PPS_MAX_SIZE];
    td_u32 sps_len = 0;
    td_u32 pps_len = 0;
    char path[256];
    td_s32 ret;

    ret = nal_get_sps_pps(sps, &sps_len, pps, &pps_len);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }

    dms_clip_make_path(path, sizeof(path));
    g_clip_writer = mp4_open(path, APP_VPSS_OUT_WIDTH, APP_VPSS_OUT_HEIGHT,
                             sps, sps_len, pps, pps_len);
    if (g_clip_writer == NULL) {
        printf("dms_clip: open %s failed\n", path);
        return TD_FAILURE;
    }

    g_clip_stop_pts_us = frame->pts + (td_u64)APP_DMS_CLIP_POST_SECONDS * 1000000ULL;
    /* 回退模式没有预录，直接把这个新 IDR 当第一帧；它就是后续帧的参考图。 */
    ret = mp4_write_frame(g_clip_writer, frame->data, frame->len, frame->pts);
    if (ret != TD_SUCCESS) {
        printf("dms_clip: write first frame failed, closing %s\n", path);
        (void)mp4_close(g_clip_writer);
        g_clip_writer = NULL;
        return TD_FAILURE;
    }

    printf("dms_clip: recording started (fallback, no pre-roll) -> %s\n", path);
    return TD_SUCCESS;
}

static td_s32 dms_clip_on_frame(const media_frame *frame, void *arg)
{
    td_bool requested;
    td_bool newest_is_idr;
    td_s32 ret = TD_SUCCESS;

    (void)arg;

    pthread_mutex_lock(&g_clip_lock);

    /*
     * [步骤 1/3] 不管是不是在录像，先把这一帧拷进环形缓冲——触发前录像全靠它。
     *    注意：回调返回后 media_source 会释放原始 data，所以必须是拷贝。
     */
    dms_clip_ring_push(frame);

    newest_is_idr = TD_FALSE;
    if (g_ring_count > 0) {
        newest_is_idr = g_ring[(g_ring_head + g_ring_count - 1) % DMS_CLIP_RING_SLOTS].is_idr;
    }

    /* [步骤 2/3] 触发请求只置了标志，这里在回调线程里真正开文件、延长停止时间或写实时帧。 */
    requested = atomic_load(&g_clip_requested);
    if (requested) {
        atomic_store(&g_clip_requested, TD_FALSE);
    }

    if (g_clip_writer == NULL) {
        if (requested) {
            /* 触发：优先从环形缓冲回写"触发前"的画面。 */
            ret = dms_clip_begin_from_ring(frame);
            if (ret != TD_SUCCESS) {
                /*
                 * 找不到合格的 IDR / 拿不到 SPS/PPS / 开文件失败：回退到等下一个 IDR。
                 * 失败也必须清掉“本周期已触发”，下一次事件才能重新置 requested。
                 */
                g_clip_wait_idr = TD_TRUE;
                g_clip_request_once = TD_FALSE;
                g_clip_last_request_ms = dms_clip_now_ms();
                printf("dms_clip: no usable pre-roll IDR, wait for next IDR\n");
            }
        } else if (g_clip_wait_idr == TD_TRUE && newest_is_idr != TD_FALSE) {
            /* 回退模式生效：这个 IDR 到了，直接开文件写它。 */
            ret = dms_clip_begin_fallback(frame);
            if (ret == TD_SUCCESS) {
                g_clip_wait_idr = TD_FALSE;
            } else {
                /* 回退开文件也失败：记录本轮结束时刻，并允许后续事件重新发起。 */
                g_clip_request_once = TD_FALSE;
                g_clip_last_request_ms = dms_clip_now_ms();
            }
        }
    } else {
        if (requested) {
            /* 录像中途又触发：把停止时间往后延（不重开文件）。 */
            g_clip_stop_pts_us = frame->pts +
                                 (td_u64)APP_DMS_CLIP_POST_SECONDS * 1000000ULL;
            printf("dms_clip: new event during recording, stop extended\n");
        }
        ret = mp4_write_frame(g_clip_writer, frame->data, frame->len, frame->pts);
        if (ret != TD_SUCCESS) {
            printf("dms_clip: write frame failed, closing\n");
            (void)mp4_close(g_clip_writer);
            g_clip_writer = NULL;
            g_clip_request_once = TD_FALSE;
            g_clip_last_request_ms = dms_clip_now_ms();
        }
    }

    /* [步骤 3/3] 刚开完文件（已写完回写段）或正在录：检查是否到点收尾。 */
    if (g_clip_writer != NULL && frame->pts >= g_clip_stop_pts_us) {
        printf("dms_clip: recording finished, %u frame(s)\n",
               mp4_frame_count(g_clip_writer));
        (void)mp4_close(g_clip_writer);
        g_clip_writer = NULL;
        /* 正常结束才是下一次冷却的起点；保留已触发标记，让紧接着的事件先被冷却拦住。 */
        g_clip_last_request_ms = dms_clip_now_ms();
    }

    pthread_mutex_unlock(&g_clip_lock);
    return ret;
}

td_s32 dms_clip_start(void)
{
    if (g_clip_subscription != NULL) {
        return TD_SUCCESS;
    }
    g_clip_subscription = media_source_subscribe(dms_clip_on_frame, NULL);
    if (g_clip_subscription == NULL) {
        printf("dms_clip: subscribe failed\n");
        return TD_FAILURE;
    }
    return TD_SUCCESS;
}

/* 输入：无。输出：停止订阅并关闭未完成的 MP4。注意：会清零冷却和待触发标志。 */
void dms_clip_stop(void)
{
    if (g_clip_subscription != NULL) {
        media_source_unsubscribe(g_clip_subscription);
        g_clip_subscription = NULL;
    }
    pthread_mutex_lock(&g_clip_lock);
    if (g_clip_writer != NULL) {
        (void)mp4_close(g_clip_writer);
        g_clip_writer = NULL;
    }
    dms_clip_ring_reset();
    g_clip_wait_idr = TD_FALSE;
    g_clip_request_once = TD_FALSE;
    g_clip_last_request_ms = 0.0;
    g_clip_last_cooldown_log_ms = -1.0;
    atomic_store(&g_clip_requested, TD_FALSE);
    pthread_mutex_unlock(&g_clip_lock);
}

#else /* APP_DMS_CLIP_ENABLED */

/* 录像关闭时给 dms_thread.c 一个空实现，避免条件编译散落在调用方。 */
td_s32 dms_clip_start(void)
{
    return TD_FAILURE;
}

void dms_clip_stop(void)
{
}

void dms_clip_request(void)
{
}

#endif /* APP_DMS_CLIP_ENABLED */
