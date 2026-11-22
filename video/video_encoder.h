/*
 * 【说明】video_encoder：负责 VENC（Video Encode，视频编码）。
 *   VPSS 出来的是 NV21 原始图像（1280x720 一帧约 1.3 MB，30 帧/秒就是约 40 MB/秒，没法存也没法发）；
 *   编码器把它压缩成 H.264 码流（本项目设为约 2 Mbit/s ≈ 0.25 MB/秒），压缩了 100+ 倍。
 *
 *   本模块有两类函数：
 *    (1) 编码通道管理：create / start_receive / bind_vpss（以及相反的 unbind / stop / destroy）
 *    (2) 取流线程：start_stream 开线程不断把编码好的码流"取出来"，
 *        写到文件，同时推给 media_source（再由它分发给 RTSP/RTP 等消费者）
 */

/**
 * @file video_encoder.h
 * @brief H.264编码通道、VPSS连接及后台码流保存接口。
 */
#ifndef VIDEO_ENCODER_H
#define VIDEO_ENCODER_H

#include <stdio.h>
#include "ot_type.h"

/* 配置并创建 H.264 编码通道 0。 */
/*
 * 【说明】create 只是创建并配置通道；还没开始接收图像，要 start_receive 之后才会收。
 */
td_s32 video_encoder_create(void);

/* 启动编码通道 0，持续接收图像。 */
td_s32 video_encoder_start_receive(void);

/* 将 VPSS Group 0 Channel 0 绑定到 VENC Channel 0。 */
td_s32 video_encoder_bind_vpss(void);

/* 获取并保存一帧编码码流。 */
/*
 * 【说明】一次取走一帧的码流并处理。取流线程会反复调用它。
 *   它是整个"编码器 → 网络"的入口，见 video_encoder.c 里的详细说明。
 */
td_s32 video_encoder_save_one_frame(FILE *file);

/* 打开输出文件并创建后台取流线程。 */
td_s32 video_encoder_start_stream(const char *file_path);

/* 请求取流线程退出，等待结束后关闭文件。 */
td_s32 video_encoder_stop_stream(void);

/* 在线程结束后，查询取流和文件写入是否成功。 */
td_s32 video_encoder_get_stream_result(void);

/* 解除 VPSS 到 VENC 的绑定。 */
td_s32 video_encoder_unbind_vpss(void);

/* 停止编码通道 0 接收图像。 */
td_s32 video_encoder_stop_receive(void);

/* 销毁编码通道 0。 */
td_s32 video_encoder_destroy(void);

#endif
