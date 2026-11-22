/*
 * 【说明】video_capture：负责"从镜头传感器到 VI 通道"这一段（管线最前端）。
 *   数据流向（一帧图像的诞生）：
 *      Sensor(SC4336P 传感器，输出 RAW 原始数据)
 *         → MIPI RX(接收传感器通过 MIPI 高速串行线发来的数据)
 *         → VI Dev(视频输入设备，把 MIPI 数据接进芯片)
 *         → VI Pipe(视频输入管线，里面还会经过 ISP 做图像处理)
 *         → VI Chn(视频输入通道，最终输出 NV21 格式的 2560x1440 图像)
 *   启动顺序是上面从上到下，停止顺序是从下到上。main.c 里的 goto 清理标签就是按这个对应的。
 */

/**
 * @file video_capture.h
 * @brief MIPI、VI设备、VI Pipe和VI Channel采集控制接口。
 */
#ifndef VIDEO_CAPTURE_H
#define VIDEO_CAPTURE_H

#include "ot_type.h"

/* 设置 MIPI 接收器的数据 lane 分组方式。 */
/*
 * 【说明】lane = MIPI 的"数据通道"（一对差分线）。传感器有 2 条 lane，芯片最多支持 4 条，
 *   先要告诉芯片这些 lane 怎么分组，之后才能配置具体怎么收。
 */
td_s32 video_capture_set_mipi_lane_mode(void);

/** 使能传感器时钟并释放传感器复位。 */
td_s32 video_capture_start_sensor(void);

/* 配置并启动 MIPI 接收器。 */
td_s32 video_capture_start_mipi(void);

/* 配置并启动 VI 设备。 */
td_s32 video_capture_start_vi_dev(void);

/* 绑定 VI Dev 与 VI Pipe。 */
td_s32 video_capture_bind_pipe(void);

/* 设置 VI Group 的WDR融合属性。 */
td_s32 video_capture_set_vi_group(void);

/* 创建并启动 VI Pipe。 */
td_s32 video_capture_start_pipe(void);

/* 配置并使能 VI Channel 0。 */
td_s32 video_capture_start_chn(void);

/* 获取并释放一帧，用于验证 VI 通道是否输出图像。 */
/*
 * 【说明】调试用：向 VI 通道要一帧再还回去，用来确认"传感器→VI 这条链路真的出图了"。
 *   main.c 当前没有调用它，但排错时很有用。
 */
td_s32 video_capture_test_frame(void);

/* 从 VI 通道获取一帧并保存为无压缩的 NV21 文件。 */
td_s32 video_capture_save_yuv(const char *file_path);

/* 禁用 VI Channel 0。 */
void video_capture_stop_chn(void);

/* 停止并销毁 VI Pipe。 */
void video_capture_stop_pipe(void);

/* 解除 VI 设备与 VI Pipe 的绑定。 */
void video_capture_unbind_pipe(void);

/* 停止 VI 设备。 */
void video_capture_stop_vi_dev(void);

/* 复位并关闭 MIPI 接收器。 */
void video_capture_stop_mipi(void);

/** 复位传感器并关闭传感器时钟。 */
void video_capture_stop_sensor(void);

#endif
