/*
 * DMS 结果的输出接口。dms_thread 每次更新状态后可以发送叠加文字，
 * 每次产生新事件时调用 dms_sink_notify_event。该函数写事件日志，
 * 通知已注册的事件回调，再请求内置事件录像。
 *
 * main.c 在 DMS 初始化后接入 video_osd。车速由已注册的回调提供，
 * 板端未接车速时使用 -1 表示未知。
 */
#include <stdio.h>

#include "app_config.h"
#include "dms_sink.h"
#include "dms_log.h"
#include "dms_clip.h"

static dms_overlay_cb g_overlay_cb = NULL;
static void *g_overlay_arg = NULL;
static dms_clip_cb g_clip_cb = NULL;
static void *g_clip_cb_arg = NULL;
static dms_speed_cb g_speed_cb = NULL;
static void *g_speed_arg = NULL;

/* 最近一次叠加文字：收到就缓存一份，video_osd 之类模块可以取走。 */
static char g_overlay_text[DMS_OVERLAY_TEXT_MAX];

td_s32 dms_sink_init(void)
{
    g_overlay_cb = NULL;
    g_overlay_arg = NULL;
    g_clip_cb = NULL;
    g_clip_cb_arg = NULL;
    g_speed_cb = NULL;
    g_speed_arg = NULL;
    g_overlay_text[0] = '\0';
    return TD_SUCCESS;
}

/* 输入：无。输出：释放 sink 的回调引用。注意：必须在停止 DMS 线程后调用。 */
void dms_sink_deinit(void)
{
    g_overlay_cb = NULL;
    g_clip_cb = NULL;
    g_speed_cb = NULL;
}

/* 输入：文字回调及调用方私有参数。输出：后续状态文字会转发给该回调。 */
void dms_sink_set_overlay_hook(dms_overlay_cb cb, void *arg)
{
    g_overlay_cb = cb;
    g_overlay_arg = arg;
}

/* 输入：事件录像回调及私有参数。输出：每次事件先通知此回调。注意：内部 MP4 录像仍由 sink 管理。 */
void dms_sink_set_clip_hook(dms_clip_cb cb, void *arg)
{
    g_clip_cb = cb;
    g_clip_cb_arg = arg;
}

/* 输入：车速提供回调及私有参数。输出：注册后状态日志可附带车速。 */
void dms_sink_set_speed_provider(dms_speed_cb cb, void *arg)
{
    g_speed_cb = cb;
    g_speed_arg = arg;
}

/* 输入：无。输出：当前车速 km/h；未接 CAN 时返回 -1.0。 */
float dms_sink_get_speed_kmh(void)
{
    if (g_speed_cb != NULL) {
        return g_speed_cb(g_speed_arg);
    }
    return -1.0f;   /* 没有接入 CAN 车速 */
}

/* 输入：一条报警事件。输出：事件日志，并通知外部回调、请求事件录像。 */
void dms_sink_notify_event(const dms_event *ev)
{
    if (ev == NULL || ev->type == DMS_EVENT_NONE) {
        return;
    }
    dms_log_event(ev);

    if (g_clip_cb != NULL) {
        g_clip_cb(ev, g_clip_cb_arg);
    }
    /* 这里只发送触发请求。dms_clip 的码流订阅回调负责缓存和写文件。 */
    dms_clip_request();
}

/*
 * 把状态文字交给叠加回调（main.c 会接到 video/video_osd.c），同时缓存一份最新的。
 * 没接回调时也只是缓存，不影响调用方。画字实现见 video/video_osd.c。
 */
void dms_sink_overlay(const char *text)
{
    if (text != NULL) {
        snprintf(g_overlay_text, sizeof(g_overlay_text), "%s", text);
    }
    if (g_overlay_cb != NULL) {
        g_overlay_cb(text, g_overlay_arg);
    }
}

const char *dms_sink_get_overlay_text(void)
{
    return g_overlay_text;
}
