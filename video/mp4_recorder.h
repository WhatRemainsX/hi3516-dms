/**
 * @file mp4_recorder.h
 * @brief MP4 录像模块：订阅 media_source，自动等待关键帧，录指定时长后封好文件。
 *
 * 它只负责"录像流程"（什么时候开始、录多久、什么时候收尾），
 * 怎样把帧写成 MP4 文件格式，是 mp4_writer 的事。
 */
#ifndef MP4_RECORDER_H
#define MP4_RECORDER_H

#include "ot_type.h"

/* 不透明类型：外部只拿指针，成员定义在 mp4_recorder.c 里 */
typedef struct mp4_recorder mp4_recorder;

/*
 * 开始录像。
 *
 * file_path     输出文件路径，例如 "/mnt/sd/nfs/rec.mp4"
 * video_width / video_height   画面大小（像素），写进 MP4 的头部
 * seconds       录多少秒后自动封好文件；0 表示一直录，直到调用 mp4_recorder_stop
 *
 * 返回：成功返回句柄，失败返回 NULL。
 * 注意：调用返回时文件还没有创建。真正开始写文件要等到第一个关键帧（IDR），
 *       所以会有最多约 2 秒（一个 GOP）的延迟。
 */
mp4_recorder *mp4_recorder_start(const char *file_path, td_u32 video_width,
                                 td_u32 video_height, td_u32 seconds);

/*
 * 停止录像：取消订阅；如果文件还没封好（比如录到一半被 Ctrl+C）就先封好；最后释放。
 * 允许传 NULL，调用后 recorder 指针不能再使用。
 */
void mp4_recorder_stop(mp4_recorder *recorder);

#endif
