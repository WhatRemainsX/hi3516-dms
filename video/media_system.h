/*
 * 【说明】media_system：整个程序最先要初始化的"地基"。
 *   海思芯片里图像是在"硬件模块"之间流动的（VI→VPSS→VENC），
 *   流动的图像放在一块"公共内存池"里（叫 VB，Video Buffer）。
 *   所以第一件事：划好内存池，再让 SYS 模块工作起来。
 *   本文件只声明两个函数，具体实现在 media_system.c。
 */

/**
 * @file media_system.h
 * @brief 视频缓存池及媒体系统的初始化和反初始化接口。
 */
#ifndef MEDIA_SYSTEM_H
#define MEDIA_SYSTEM_H

#include "ot_type.h"

/* 配置公共视频缓存池、VI/VPSS工作模式并初始化媒体系统。 */
/*
 * 【说明】td_s32 是海思定义的"有符号 32 位整数"（等价于 int）。
 *   返回 0（TD_SUCCESS）表示成功，非 0 表示失败。
 *   这是海思 SDK 的通用习惯：几乎所有函数都用返回值告诉你成败。
 */
td_s32 media_system_init(void);

/* 退出媒体系统并释放公共视频缓存池。 */
void media_system_deinit(void);

#endif
