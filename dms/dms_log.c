/*
 * dms_log.c：dms_log.h 的实现（从 dms_sink.c 拆出）。
 * 输入：dms_status 状态快照、dms_event 事件。输出：打印到标准输出的状态/事件日志。
 */
#include <stdio.h>

#include "dms_log.h"

/* 输入：报警级别枚举。输出：日志用的固定英文短名。 */
const char *dms_level_text(dms_alarm_level level)
{
    switch (level) {
    case DMS_ALARM_NOTICE:  return "NOTICE";
    case DMS_ALARM_WARNING: return "WARNING";
    case DMS_ALARM_STRONG:  return "STRONG";
    case DMS_ALARM_RMF:     return "RMF";
    default:                return "NONE";
    }
}

/* 输入：一份完整 DMS 状态和当前车速（-1 表示没有车速）。输出：一行便于查找的状态日志。注意：本函数只读状态。 */
void dms_log_status(const dms_status *st, float speed)
{

    if (st == NULL) {
        return;
    }
    /* yaw/pitch/roll 是相对摄像头的原始值；rel 是减去本人正常姿态基线后的值（分心按它判）。 */
    printf("dms: %.1fs level=%s EAR=%.3f PERCLOS=%.3f MAR=%.3f eye=%s "
           "yaw=%.1f pitch=%.1f roll=%.1f yawn=%u%s speed=%.1fkm/h "
           "rel_yaw=%.1f rel_pitch=%.1f pose_cal=%d ear_base=%.3f eor=%u\n",
           st->t_ms / 1000.0, dms_level_text(st->level),
           st->ear, st->perclos, st->mar,
           st->eye == DMS_EYE_OPEN ? "open" : (st->eye == DMS_EYE_CLOSED ? "closed" : "unknown"),
           st->yaw, st->pitch, st->roll, st->yawn_count,
           st->distracted ? " DISTRACT" : "",
           speed, st->yaw_rel, st->pitch_rel, st->pose_cal == TD_TRUE ? 1 : 0, st->ear_base,
           st->eor_stage);
}

/* 输入：一条报警事件。输出：一行事件日志。 */
void dms_log_event(const dms_event *ev)
{
    if (ev == NULL) {
        return;
    }
    printf("dms_event: %.1fs type=%d level=%s %s\n",
           ev->t_ms / 1000.0, (int)ev->type, dms_level_text(ev->level), ev->desc);
}
