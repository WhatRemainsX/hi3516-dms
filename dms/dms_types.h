/*
 * dms_types.h：DMS（驾驶员监控）模块的公共类型。
 * 流水线位置：贯穿 DMS 全链路的“数据字典”，本身不处理图像。
 * 输入/输出：各模块用它交换检测框、关键点、状态和事件，不产生运行结果。
 * 调用关系：dms_detect/models/metrics/sink/thread 都包含本头文件。
 *   只放数据结构，不含实现。dms_detect / dms_models / dms_metrics / dms_sink /
 *   dms_thread 都包含这个头。
 *
 * 坐标系约定：
 *   所有像素坐标都在 VPSS 通道 2 的 640x640 画面里（APP_DMS_IN_WIDTH/HEIGHT）。
 *   检测框、关键点、眼睛框都用这个坐标系，不再换算回 1280x720。
 */
#ifndef DMS_TYPES_H
#define DMS_TYPES_H

#include "ot_type.h"

/* 98 点关键点模型的点数（WFLW 98 点，PFLD 输出 1x196 = 98*(x,y)）。 */
#define DMS_LANDMARK_COUNT 98

/* 最多跟几张脸。驾驶员监控一般只有 1 张，留几张是为了误检时不崩。 */
#define DMS_MAX_FACES 4

/* 报警级别：无 / 轻提醒 / 警告 / 强提醒。 */
typedef enum {
    DMS_ALARM_NONE = 0,
    DMS_ALARM_NOTICE,
    DMS_ALARM_WARNING,
    DMS_ALARM_STRONG,
    DMS_ALARM_RMF               /* L4：最小风险策略（GB 47955） */
} dms_alarm_level;

/* 眼睛状态。UNKNOWN 表示这一帧没有有效关键点或没跑分类器。 */
typedef enum {
    DMS_EYE_UNKNOWN = 0,
    DMS_EYE_OPEN,
    DMS_EYE_CLOSED
} dms_eye_state;

/* 事件类型。 */
typedef enum {
    DMS_EVENT_NONE = 0,
    DMS_EVENT_FATIGUE_PERCLOS,   /* PERCLOS 超阈值 */
    DMS_EVENT_FATIGUE_BLINK,     /* 长闭眼 */
    DMS_EVENT_YAWN,              /* 打哈欠次数超阈值 */
    DMS_EVENT_DISTRACT,          /* 头姿分心 */
    DMS_EVENT_OCCLUSION,         /* 摄像头遮挡 */
    DMS_EVENT_EOR,               /* L1 视线脱离提示 */
    DMS_EVENT_EOR_UPGRADE,       /* L2 升级 EOR */
    DMS_EVENT_DCA,               /* L3 驾驶员接管需求 */
    DMS_EVENT_RMF                /* L4 最小风险策略 */
} dms_event_type;

/* 一个像素点。 */
typedef struct {
    float x;
    float y;
} dms_point;

/* 一个矩形框：x1,y1 左上，x2,y2 右下，score 检测置信度。
 * 这套字段也用于跟踪框和外扩框；坐标必须保持同一套 640x640 像素单位。 */
typedef struct {
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
} dms_box;

/* 98 个关键点，下标按 WFLW 98 点分组（PFLD 98 点）：
 *   轮廓 0-32；右眉 33-41；左眉 42-50；鼻中线 51-54；鼻底线 55-59；
 *   右眼 60-67；左眼 68-75；外唇 76-87（嘴角 76/82）；内唇 88-95（上中 90、下中 94）；
 *   右瞳孔 96；左瞳孔 97。 */
typedef struct {
    dms_point pt[DMS_LANDMARK_COUNT];
} dms_landmarks;

/* 一帧里认出来的一张脸的全部信息。
 * 符号约定与 dms_models 的 PnP 一致（均按画面方向）：yaw>0 鼻尖朝画面右，
 * pitch>0 抬头（<0 低头），roll>0 画面顺时针。 */
typedef struct {
    dms_box box;                 /* 外扩后的框（仅记录；关键点裁剪用跟踪原始框） */
    dms_landmarks lm;            /* 98 点，640x640 坐标 */
    td_bool lm_valid;            /* 关键点是否有效 */
    float ear_left;              /* 左眼 EAR（无效时为 -1） */
    float ear_right;             /* 右眼 EAR */
    float ear;                   /* 双眼平均 EAR；与分类器联合参与闭眼判定（见 dms_metrics.c） */
    td_bool ear_valid;           /* 已废弃，判定不再读取，保留只为兼容 */
    float mar;                   /* 嘴纵横比 */
    dms_eye_state eye_left;      /* 左眼分类结果 */
    dms_eye_state eye_right;     /* 右眼分类结果 */
    dms_eye_state eye;           /* 分类器粗略综合（任一只闭即闭），最终判定见 dms_status.eye */
    float yaw;                   /* 头姿 yaw，度 */
    float pitch;                 /* 头姿 pitch，度 */
    float roll;                  /* 头姿 roll，度 */
    td_bool pose_valid;          /* 头姿是否有效 */
    float speed_kmh;             /* 车速 km/h；未接车速为 -1 */
} dms_face_state;

/*
 * 环形缓冲里的一个采样点（约 12fps，60 秒约 720 个）。
 * 每个样本只记“当时是否闭眼/哈欠/分心”和 EAR；窗口统计靠 t_ms 往回筛，
 * 不复制原图，所以内存很小。
 */
typedef struct {
    double t_ms;                 /* 采样时刻，单调时钟毫秒 */
    td_bool eye_closed;          /* 这一帧是否闭眼 */
    td_bool yawn;                /* 这一帧是否处于打哈欠 */
    td_bool distracted;          /* 这一帧是否处于分心 */
    float ear;                   /* 当时的 EAR（无效为 -1） */
} dms_sample;

/* 一次报警事件。
 * type 说明类别，level 说明当时报警强度；t_ms 用单调时钟，避免系统校时跳动。 */
typedef struct {
    dms_event_type type;
    dms_alarm_level level;
    double t_ms;
    char desc[64];
} dms_event;

/* 给日志/叠加用的当前状态快照。
 * 这是“本帧结论”，不负责历史统计；PERCLOS/哈欠窗口由 dms_metrics 维护。 */
typedef struct {
    double t_ms;
    dms_alarm_level level;
    float ear;
    float mar;
    float perclos;               /* 60 秒窗口内的闭眼占比，0~1 */
    dms_eye_state eye;
    float yaw;                   /* 相对摄像头的原始头姿（度） */
    float pitch;
    float roll;
    float yaw_rel;               /* 减去本人正常姿态基线后的头姿（分心按它判） */
    float pitch_rel;
    td_bool pose_cal;            /* 头姿基线是否已标定好（未标定前不判分心） */
    float ear_base;              /* 本人睁眼 EAR 基线 */
    td_u32 yawn_count;           /* 60 秒窗口内打哈欠次数 */
    td_bool distracted;
    td_bool perclos_fatigue;     /* PERCLOS 超阈值（带滞回） */
    td_bool yawn_fatigue;        /* 窗口内哈欠次数达到阈值 */
    td_bool long_closure;        /* 当前正处于长闭眼（闭眼已超过 APP_DMS_LONG_BLINK_MS） */
    td_bool fatigue_alert;       /* 疲劳报警进行中（PERCLOS 超标 或 哈欠过多） */
    td_bool face_present;        /* 本帧是否有人脸 */
    td_u32 eor_stage;            /* 视线脱离链当前阶段：0 无，1 EOR，2 升级 EOR，3 DCA，4 RMF */
    td_u32 face_count;           /* 当前跟踪到的人脸数 */
} dms_status;

#endif /* DMS_TYPES_H */
