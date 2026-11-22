/*
 * dms_sink：DMS 的输出侧接口（总入口）。
 *   - RTSP 叠加（缓存最新文字 + 回调钩子，由 main.c 接到 video/video_osd.c）
 *   - 报警事件分发：写日志（dms_log）+ 外部回调 + 触发事件录像（dms_clip）
 *   - CAN 车速接入点（接口预留 + TODO）
 * 其余输出拆在独立文件里：
 *   - dms_log.c       状态日志、事件日志
 *   - dms_clip.c      事件录像
 *   - dms_occlusion.c 摄像头遮挡自检
 *
 * 设计原则：DMS 的输出全部经过这里，任何一个 sink 出错都只影响 DMS，
 * 不会影响采图/编码/RTSP/录像主流程。
 */
#ifndef DMS_SINK_H
#define DMS_SINK_H

#include "dms_types.h"

/* 叠加文字的最大长度（含结尾 '\0'）。dms_thread 生成文字、video_osd 显示都用它。 */
#define DMS_OVERLAY_TEXT_MAX 96

/* RTSP 叠加回调：text 是要显示的文字（含 EAR/PERCLOS 等）。 */
typedef void (*dms_overlay_cb)(const char *text, void *arg);

/* 事件回调：在报警事件产生时调用。 */
typedef void (*dms_clip_cb)(const dms_event *ev, void *arg);

/* 车速回调：返回 km/h；返回 < 0 表示没有车速信号。 */
typedef float (*dms_speed_cb)(void *arg);

td_s32 dms_sink_init(void);
void dms_sink_deinit(void);

void dms_sink_set_overlay_hook(dms_overlay_cb cb, void *arg);
void dms_sink_set_clip_hook(dms_clip_cb cb, void *arg);
void dms_sink_set_speed_provider(dms_speed_cb cb, void *arg);

/* 取当前车速，没有接入时返回 -1。 */
float dms_sink_get_speed_kmh(void);

/*
 * 把一行状态文字交给叠加回调，同时缓存一份最新的（dms_sink_get_overlay_text 可取）。
 * main.c 会把回调接到 video/video_osd.c；没接回调时也只是缓存，不影响调用方。
 */
void dms_sink_overlay(const char *text);

/* 取最近一次 dms_sink_overlay() 缓存的文字（没有则返回空串）。 */
const char *dms_sink_get_overlay_text(void);

/* 报警事件：打印 + 调用 clip hook + 触发事件录像。 */
void dms_sink_notify_event(const dms_event *ev);

#endif /* DMS_SINK_H */
