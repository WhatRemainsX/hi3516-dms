/*
 * dms_metrics：疲劳/分心的判定指标和状态机。
 *   - EAR：标准 6 点公式（需要每只眼 6 个关键点）
 *   - MAR：嘴纵横比，用 98 点（WFLW）的 p76/p82（嘴角）和 p90/p94（内唇上下中）
 *   - PERCLOS：60 秒环形缓冲里闭眼样本占比
 *   - 长闭眼 / 打哈欠 / 头姿分心：时间状态机
 *   - 报警三级：按异常持续时间升级
 */
#ifndef DMS_METRICS_H
#define DMS_METRICS_H

#include "dms_types.h"

/* 每帧最多产生的事件数（dms_thread 的事件数组按它分配）。 */
#define DMS_MAX_EVENTS 8

/* 60 秒窗口内最多记多少次打哈欠（正常不会超过）。 */
#define DMS_YAWN_MAX 16

/* 环形缓冲：存最近 APP_DMS_PERCLOS_WIN_MS 毫秒的采样。 */
typedef struct {
    dms_sample *buf;
    td_u32 capacity;
    td_u32 head;      /* 下一个写入位置 */
    td_u32 count;     /* 已写入的样本数（最多 capacity） */
} dms_ring;

typedef struct {
    dms_ring ring;

    /* 打哈欠 */
    td_bool yawn_open;
    td_bool yawn_counted;
    double yawn_begin_ms;
    double yawn_last_high_ms;    /* 最后一次 MAR 超门槛的时刻（哈欠去抖用） */
    double yawn_times[DMS_YAWN_MAX];
    td_u32 yawn_count;

    /* 长闭眼 */
    td_bool blink_open;
    td_bool blink_counted;
    double blink_begin_ms;
    double blink_last_closed_ms;   /* 最近一次判闭眼的时刻（去抖用） */
    td_u32 blink_open_frames;      /* 闭眼段中连续判睁的帧数（去抖用） */

    /* 分心 */
    td_bool distract_open;
    td_bool distracted;
    double distract_begin_ms;

    /* 睁眼 EAR 基线（闭眼判定用） */
    float ear_base;
    td_u32 ear_base_n;
    float ear_init[24];          /* 基线初值：前 24 个睁眼样本取中位数（抗离群值） */

    /* 疲劳报警持续计时 */
    td_bool fatigue_open;
    double fatigue_begin_ms;

    /* 视线脱离链 */
    td_bool face_seen;           /* 本次运行是否见过人脸（无人脸算视线脱离的前提） */
    double noface_since_ms;      /* 本段无人脸开始时刻，0 表示当前有人脸 */
    td_bool eor_open;
    double eor_begin_ms;
    td_bool eor_latched;         /* 视线脱离（带复位延时）的当前值 */
    double eor_on_since_ms;      /* 视线回到路面的开始时刻，0 表示还没回来 */
    td_u32 eor_stage;

    /* 头姿基线（分心判定用：摄像头不在正前方时，正常坐姿也有固定的 yaw/pitch） */
    float pose_base_yaw;
    float pose_base_pitch;
    td_u32 pose_base_n;
    double last_face_ms;
    double first_sample_ms;    /* PERCLOS 窗口第一份样本时刻；数据不满半个窗口不判 PERCLOS */

    /* 报警状态 */
    dms_alarm_level level;

    /* 边沿检测用 */
    td_bool prev_perclos_fatigue;
    td_bool prev_yawn_fatigue;
    td_bool prev_distracted;
} dms_metrics;

void dms_metrics_init(dms_metrics *m);
void dms_metrics_deinit(dms_metrics *m);

/* 无人脸的帧调用：维持报警状态机（曾有人脸后连续无人脸 3.5 秒算视线脱离）。 */
void dms_metrics_process_noface(dms_metrics *m, double now_ms, dms_status *st,
                                dms_event *events, td_u32 *event_count);

/* 人脸跟丢时调用：结束进行中的长闭眼/哈欠/分心计时段，报警电平回到 NONE。 */
void dms_metrics_on_face_lost(dms_metrics *m);

/* 标准 6 点 EAR：p0..p5 按眼部轮廓顺序。
 * EAR = (|p1-p5| + |p2-p4|) / (2 * |p0-p3|)。 */
float dms_metrics_ear(const dms_point p[6]);

/* 98 点模型的 MAR：嘴角 p76/p82，内唇上中 p90 / 下中 p94。 */
float dms_metrics_mar(const dms_landmarks *lm);

/* 98 点模型的双眼 EAR：从每只眼 8 点里选 6 点调用 dms_metrics_ear()。
 * 左眼 68-75、右眼 60-67；结果写 *ear_left / *ear_right，无效时为 -1。 */
void dms_metrics_ear_98(const dms_landmarks *lm, float *ear_left, float *ear_right);

/* 处理一帧的人脸状态，更新指标，填充状态快照，并产生边沿事件。 */
void dms_metrics_process(dms_metrics *m, const dms_face_state *fs, double now_ms,
                         dms_status *st, dms_event *events, td_u32 *event_count);

#endif /* DMS_METRICS_H */
