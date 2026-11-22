/*
 * dms_log：DMS 的状态日志和事件日志。
 */
#ifndef DMS_LOG_H
#define DMS_LOG_H

#include "dms_types.h"

/* 报警级别文字（NONE/NOTICE/WARNING/STRONG/RMF）。 */
const char *dms_level_text(dms_alarm_level level);

/* 周期状态日志（每秒一行）；speed_kmh < 0 表示没有车速信号。 */
void dms_log_status(const dms_status *st, float speed_kmh);

/* 报警事件日志（一条事件一行）。 */
void dms_log_event(const dms_event *ev);

#endif /* DMS_LOG_H */
