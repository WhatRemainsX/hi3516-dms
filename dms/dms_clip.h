/*
 * dms_clip：DMS 事件录像（触发前 N 秒环形缓冲 + 触发后 N 秒）。
 *   常驻订阅 media_source 的 H.264 码流，报警事件到来时把预录 + 后续画面写成 MP4。
 *   APP_DMS_CLIP_ENABLED 为 0 时三个函数都是空实现，调用方不用加条件编译。
 */
#ifndef DMS_CLIP_H
#define DMS_CLIP_H

#include "ot_common.h"

/* 开始订阅码流（由 dms_thread 启动时调用）。录像关闭时返回 TD_FAILURE。 */
td_s32 dms_clip_start(void);

/* 取消订阅并关闭未写完的 MP4（由 dms_thread 退出时调用）。 */
void dms_clip_stop(void);

/* 请求录一段事件录像：只置标志，真正开文件在订阅回调线程里做；有冷却时间。 */
void dms_clip_request(void);

#endif /* DMS_CLIP_H */
