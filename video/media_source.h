/*
 * H.264 码流分发接口。取流线程提交完整帧，各订阅者在独立线程中收到回调。
 * 阅读顺序：subscribe 建立订阅，push 投递一帧，unsubscribe 停止订阅。
 */
#ifndef MEDIA_SOURCE_H
#define MEDIA_SOURCE_H

#include "ot_type.h"

/* 单帧码流长度上限。取流线程按同一上限分配拼帧缓冲区。 */
#define MEDIA_FRAME_MAX_SIZE (512 * 1024)

typedef struct {
    td_u8 *data;          /* 一帧连续的 H.264 字节流。 */
    td_u32 len;           /* data 中有效的字节数。 */
    td_u64 pts;           /* 显示时间戳，单位微秒。 */
    td_bool is_frame_end; /* 帧结束标志。当前取流线程每次投递完整帧。 */
} media_frame;

/*
 * 回调在本订阅者的线程中运行。frame 和 frame->data 在回调期间有效，
 * 要把画面留到以后使用时，请复制 data。arg 是订阅时传入的私有参数。
 */
typedef td_s32 (*media_frame_cb)(const media_frame *frame, void *arg);

/* 注册回调。成功返回订阅句柄，失败返回 NULL。 */
void *media_source_subscribe(media_frame_cb cb, void *arg);

/*
 * 取消订阅并等待该线程处理完队列。调用者应保留 arg 指向的对象直到函数返回。
 * 由外部线程用句柄注销一次。传入 NULL 可以直接返回。
 */
void media_source_unsubscribe(void *handle);

/* 为当前的订阅者复制一帧。函数返回后，调用者可以重用或释放原 data。 */
td_s32 media_source_push(const td_u8 *data, td_u32 len, td_u64 pts, td_bool is_frame_end);

/* 累计有效投递次数、各队列因满而丢弃的总帧数。可用 NULL 跳过一个值。 */
void media_source_get_stats(td_u64 *pushed, td_u64 *dropped);

#endif /* MEDIA_SOURCE_H */
