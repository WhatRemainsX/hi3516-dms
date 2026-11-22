/*
 * mp4_recorder.c：mp4_recorder.h 的实现。
 *   订阅 media_source（和调试订阅者、RTP 发送器是同一个"分发中心"），
 *   每来一帧就跑一次状态机：
 *     RECORDER_WAITING_FOR_KEY_FRAME -> （遇到关键帧）-> RECORDER_RECORDING -> （录满/出错）-> RECORDER_FINISHED
 *   为什么要等关键帧才开始录：MP4 文件必须从一个能独立解码的帧开始，否则播放器打开后前几秒花屏。
 *   这个文件只管"录像流程"，真正把帧拼成 .mp4 文件格式的工作在 mp4_writer.c 里。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mp4_recorder.h"
#include "media_source.h"
#include "nal_parser.h"
#include "mp4_writer.h"

#define NAL_TYPE_SLICE_IDR  5     /* H.264 关键帧切片的 NAL 类型编号 */

typedef enum {
    RECORDER_WAITING_FOR_KEY_FRAME,   /* 还没遇到关键帧，来的帧一律丢掉 */
    RECORDER_RECORDING,               /* 正在往文件里写 */
    RECORDER_FINISHED                 /* 已经封好文件（或出错放弃），之后来的帧直接忽略 */
} recorder_state;

struct mp4_recorder {
    char        file_path[256];                         /* 录像文件的路径 */
    td_u32      video_width;                            /* 画面宽度（像素） */
    td_u32      video_height;                           /* 画面高度（像素） */
    td_u64      max_duration_us;                        /* 最长录多久，单位微秒；0 表示不限时长 */

    recorder_state state;                               /* 当前处于哪个状态 */
    mp4_writer    *writer;            /* 开始录之前是 NULL；封好文件后又会被置回 NULL */
    td_u64         first_pts_us;      /* 第一帧的时间戳，阶段 4 用来算"已经录了多久" */

    void       *subscription;                           /* 订阅句柄：取消订阅时要用 */
};

/* 这一帧里有没有关键帧切片（NAL 类型 5）。1 = 有，0 = 没有 */
static int frame_contains_key_frame(const media_frame *frame)
{
    nal_frame_info nal_list;                            /* 拆出来的 NAL 列表 */
    td_u32 nal_index;                                   /* 遍历 NAL 列表用的下标 */
    td_s32 ret;         /* nal_parse_frame 的返回值 */

    /* 把整帧拆成一个个 NAL，结果放进 nal_list（只记录位置、长度、类型，不拷贝数据） */
    ret = nal_parse_frame(frame->data, frame->len, &nal_list);
    if (ret != TD_SUCCESS) {
        return 0;       /* 拆不开就当作"不是关键帧" */
    }
    for (nal_index = 0; nal_index < nal_list.count; nal_index++) {
        if (nal_list.nals[nal_index].type == NAL_TYPE_SLICE_IDR) {
            return 1;
        }
    }
    return 0;
}

/* 封好文件：写 moov、关闭文件，状态变成 FINISHED */
static void recorder_finish_file(mp4_recorder *recorder)
{
    td_u32 frame_count;         /* 封文件前先记下帧数：mp4_close 之后 writer 就释放了，拿不到了 */
    td_s32 ret;                 /* mp4_close 的返回值 */
    const char *result_text;    /* 打印用的结果文字 */

    frame_count = mp4_frame_count(recorder->writer);    /* 先取帧数，mp4_close 之后就取不到了 */
    ret = mp4_close(recorder->writer);                  /* 写 moov、关文件；返回值先存进 ret */

    /* mp4_close 返回后 writer 已经被释放，必须置空，防止以后又被关闭一次（重复 free） */
    recorder->writer = NULL;                            /* 指针置空，防止重复释放 */
    recorder->state = RECORDER_FINISHED;                /* 状态改成“已结束”，之后来的帧都会被忽略 */

    /* 根据 mp4_close 的结果选择要打印的文字 */
    if (ret == TD_SUCCESS) {
        result_text = "saved";                          /* 成功：打印 saved */
    } else {
        result_text = "FAILED";                         /* 失败：打印 FAILED */
    }
    printf("mp4: %s, %u frames -> %s\n", result_text, frame_count, recorder->file_path);
}

/*
 * media_source 每来一帧就会调用这个回调（在取流线程的线程里执行，不是主线程）。
 * 按 recorder->state 走状态机：已结束 -> 直接忽略；等关键帧 -> 等到了就开文件；
 * 正在录 -> 写这一帧，再看看是否该收尾。
 */
static td_s32 recorder_on_frame(const media_frame *frame, void *arg)
{
    mp4_recorder *recorder = (mp4_recorder *)arg;       /* arg 是 void*，订阅时传进来的就是 recorder，转回原类型才能访问成员 */
    td_s32 ret;         /* 各个函数调用的返回值，先存起来再判断 */

    if (recorder->state == RECORDER_FINISHED) {
        return TD_SUCCESS;                      /* 已经录完，后面的帧不用管 */
    }

    if (recorder->state == RECORDER_WAITING_FOR_KEY_FRAME) {
        td_u8 sps_data[NAL_SPS_MAX_SIZE];               /* 存放 SPS 内容的数组 */
        td_u8 pps_data[NAL_PPS_MAX_SIZE];               /* 存放 PPS 内容的数组 */
        td_u32 sps_length = 0;                          /* SPS 的实际字节数，初值 0，由 nal_get_sps_pps 填写 */
        td_u32 pps_length = 0;                          /* PPS 的实际字节数，同上 */
        int has_key_frame;      /* 这一帧里有没有关键帧：1 有，0 没有 */

        has_key_frame = frame_contains_key_frame(frame);  /* 判断这一帧是不是关键帧：1 是，0 不是 */
        if (has_key_frame == 0) {
            return TD_SUCCESS;                  /* 还没等到关键帧，丢掉这一帧 */
        }
        /* 编码线程在 push 这一帧之前，已经把它里面的 SPS/PPS 缓存好了，所以这里一定取得到 */
        ret = nal_get_sps_pps(sps_data, &sps_length, pps_data, &pps_length);  /* 取出缓存的 SPS/PPS，长度写回 sps_length / pps_length */
        if (ret != TD_SUCCESS) {
            return TD_SUCCESS;                  /* 万一取不到，就继续等下一个关键帧 */
        }
        recorder->writer = mp4_open(recorder->file_path, recorder->video_width,  /* 创建 MP4 文件，返回的句柄存进 recorder->writer */
                                    recorder->video_height,
                                    sps_data, sps_length, pps_data, pps_length);
        if (recorder->writer == NULL) {
            printf("mp4: open %s failed, recording abandoned\n", recorder->file_path);
            recorder->state = RECORDER_FINISHED;    /* 放弃，避免之后每一帧都重试打开 */
            return TD_FAILURE;
        }
        recorder->first_pts_us = frame->pts;            /* 记下第一帧的时间戳，之后用来算“已经录了多久” */
        recorder->state = RECORDER_RECORDING;           /* 状态改成“正在录”，下一帧起不再走上面的等待分支 */
        printf("mp4: recording started -> %s\n", recorder->file_path);
        /* 不 return：这一帧就是关键帧，要接着往下写进文件，作为 MP4 的第一帧 */
    }

    /* 把这一帧写进 MP4 文件 */
    ret = mp4_write_frame(recorder->writer, frame->data, frame->len, frame->pts);  /* 写入这一帧，结果存进 ret */
    if (ret != TD_SUCCESS) {
        printf("mp4: write frame failed, closing file\n");
        recorder_finish_file(recorder);         /* 已经写进去的部分还能保住 */
        return TD_FAILURE;
    }

    /*
     * 检查是否录满：当前帧时间 - 第一帧时间 >= 设定时长。
     * max_duration_us 为 0 表示不限时长；
     * 先判断 pts >= first_pts_us，是为了防止时间戳异常倒退时，无符号数相减变成一个巨大的数。
     */
    if (recorder->max_duration_us != 0 && frame->pts >= recorder->first_pts_us &&
        frame->pts - recorder->first_pts_us >= recorder->max_duration_us) {
        recorder_finish_file(recorder);         /* 录满了，封好文件 */
    }
    return TD_SUCCESS;
}

mp4_recorder *mp4_recorder_start(const char *file_path, td_u32 video_width,
                                 td_u32 video_height, td_u32 seconds)
{
    mp4_recorder *recorder;

    if (file_path == NULL) {
        return NULL;
    }
    recorder = (mp4_recorder *)calloc(1, sizeof(*recorder));   /* 申请并清零 */
    if (recorder == NULL) {
        return NULL;
    }
    snprintf(recorder->file_path, sizeof(recorder->file_path), "%s", file_path);  /* 把路径拷进结构体自己的数组（最多 256 字节），不依赖调用者的字符串一直存在 */
    recorder->video_width = video_width;                /* 保存画面宽度，打开 MP4 时要用 */
    recorder->video_height = video_height;              /* 保存画面高度，打开 MP4 时要用 */
    recorder->max_duration_us = (td_u64)seconds * 1000000ULL;  /* 秒换算成微秒（1 秒 = 1000000 微秒）；先转 64 位再乘，避免溢出 */
    recorder->state = RECORDER_WAITING_FOR_KEY_FRAME;   /* 初始状态：等关键帧 */
    /* 最后才订阅：一订阅成功，回调就可能立刻在另一个线程里运行 */
    recorder->subscription = media_source_subscribe(recorder_on_frame, recorder);  /* 订阅成功会返回句柄，存起来留给 stop 用 */
    if (recorder->subscription == NULL) {
        free(recorder);                                 /* 订阅失败：释放刚申请的内存，避免泄漏 */
        return NULL;
    }

    return recorder;
}

/*
 * 停止录像：先取消订阅（保证不会再有新帧进来），再看文件有没有封好，
 * 没封好（比如还在等第一个关键帧，或者录到一半被 Ctrl+C）就在这里补上，
 * 最后释放 recorder 结构体本身。
 */
void mp4_recorder_stop(mp4_recorder *recorder)
{
    if (recorder == NULL) {
        return;
    }
    /* 第 1 步：先取消订阅。它会等回调线程完全退出，之后没有任何线程再访问 recorder 的内部状态 */
    media_source_unsubscribe(recorder->subscription);

    /* 第 2 步：如果还没封好文件就被叫停（比如按了 Ctrl+C），也要把文件封好，否则缺 moov，文件打不开 */
    if (recorder->writer != NULL) {
        recorder_finish_file(recorder);
    }
    free(recorder);                                     /* 最后释放 recorder 结构体本身 */
}

