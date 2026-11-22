/*
 * 【说明】isp_control：负责 ISP（Image Signal Processor，图像信号处理器）。
 *   传感器吐出来的是 RAW 数据（每个像素只有一种颜色、偏暗偏绿、有噪点），
 *   ISP 是芯片里的专用硬件，负责把它"加工"成正常的彩色图像：
 *      去噪 → 去坏点 → 白平衡(AWB) → 颜色校正 → 自动曝光(AE) → 伽马 → 转 YUV ……
 *   其中 AE/AWB 是"3A 算法"里的两个（还有 AF 自动对焦），它们需要不断读图像统计信息，
 *   再反过来调传感器的曝光/增益，这就是为什么 ISP 需要一个常驻线程一直运行。
 *
 *   本模块调用顺序（main.c 里）：  register → init → start
 *   退出顺序（相反）：              deinit → unregister
 */

/**
 * @file isp_control.h
 * @brief Sensor回调、AE/AWB算法及ISP工作线程控制接口。
 */
#ifndef ISP_CONTROL_H
#define ISP_CONTROL_H

#include "ot_type.h"

/* 注册 Sensor 回调、指定控制总线，并注册 AE/AWB 算法。 */
/*
 * 【说明】register = "登记"：告诉 ISP 框架
 *    ① 我用的传感器是 SC4336P（它的曝光、增益寄存器怎么写，由海思提供的 libsns_sc4336p 知道）
 *    ② 传感器的寄存器通过 I2C 总线几号控制
 *    ③ 3A 算法用海思自带的 AE 库、AWB 库
 */
td_s32 isp_control_register(void);

/* 初始化 ISP 内存、公共属性及 Sensor 模式。 */
td_s32 isp_control_init(void);

/* 创建 ISP 工作线程并进入持续运行流程。 */
/*
 * 【说明】创建一个线程，里面调用 ss_mpi_isp_run()，它会一直阻塞运行（处理每帧的中断事件），
 *   直到调用 ss_mpi_isp_exit() 才会返回。所以必须单独放进线程，否则 main 会卡死在这里。
 */
td_s32 isp_control_start(void);

/* 退出 ISP 并释放初始化资源，调用前应确保运行线程已经结束。 */
td_s32 isp_control_deinit(void);

/* 注销 AE/AWB 算法和 Sensor 回调。 */
void isp_control_unregister(void);

#endif
