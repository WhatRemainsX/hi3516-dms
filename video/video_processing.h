/*
 * 【说明】video_processing：负责 VPSS（Video Process Sub-System，视频处理子系统）。
 *   VPSS 是芯片里的图像处理硬件，可以做缩放、裁剪、旋转、降噪等。
 *   我们这里让它把 VI 出来的 2560x1440 大图"缩小"为 1280x720 小图，
 *   然后交给编码器 VENC（编码 720p 比编码 1440p 省码率、省算力、网络也更流畅）。
 *
 *   VPSS 的两级结构：
 *     Group(组)   = 一个"处理单元"，有一个输入口，接收 VI 送来的图像（所以 bind 绑的是 Group）
 *     Channel(通道)= Group 的一个"输出口"，一个 Group 可以有多个通道，各自输出不同尺寸
 *   我们只用 Group 0 的 Channel 0。
 */

/**
 * @file video_processing.h
 * @brief VPSS处理组、输出通道及其数据连接控制接口。
 */
#ifndef VIDEO_PROCESSING_H
#define VIDEO_PROCESSING_H

#include "ot_type.h"
/* ot_vpss_chn / ot_vpss_grp 等 VPSS 类型定义在 ss_mpi_vpss.h（经 ot_common.h）。
 * 头文件要自包含：只 include ot_type.h 时 ot_vpss_chn 是未定义类型，会编不过。 */
#include "ss_mpi_vpss.h"
/* 创建并启动 VPSS Group 0 及 Channel 0。 */
td_s32 video_processing_start(void);

/* 将 VI Pipe 0 Channel 0 连接到 VPSS Group 0。 */
/*
 * 【说明】bind（绑定）是海思特有的"管道连接"机制：
 *   把模块 A 的输出口直接连到模块 B 的输入口，之后图像由硬件/内核自动搬运，
 *   应用程序不需要自己取帧再送帧，既省 CPU 也没有拷贝。
 *   绑定关系：  VI(Dev0,Chn0)  ──►  VPSS(Grp0)  ──►  VENC(Chn0)
 */
td_s32 video_processing_bind_vi(void);

/* 获取并释放一帧，验证 VPSS 输出尺寸。 */
td_s32 video_processing_test_frame(void);

/* 获取 VPSS 输出的一帧并保存为 NV21 文件。 */
td_s32 video_processing_save_yuv(ot_vpss_chn vpss_chn,const char *file_path);

/* 解除 VI 到 VPSS 的连接。 */
void video_processing_unbind_vi(void);

/* 关闭输出通道，停止并销毁 VPSS Group。 */
void video_processing_stop(void);

#endif
