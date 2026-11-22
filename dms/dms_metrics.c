/*
 * dms_metrics.c：dms_metrics.h 的实现。
 * 流水线位置：DMS 链路的第三站，把关键点、眼睛和头姿结果算成疲劳/分心指标。
 * 输入：dms_face_state（EAR、MAR、眼睛状态、头姿）和单调时钟时间。
 * 输出：dms_status 状态快照和少量边沿事件；由 dms_thread 每帧调用。
 *   所有判定阈值都来自 app_config.h，便于实车标定。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "app_config.h"
#include "dms_metrics.h"

static float dms_dist(const dms_point *a, const dms_point *b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;

    return sqrtf(dx * dx + dy * dy);
}

/* 输入：6 个眼睛关键点，顺序为眼角1、上睑1、上睑2、眼角2、下睑2、下睑1。 */
/* 输出：EAR 比值；两点重合导致分母为 0 时返回 -1，表示这一帧不可用。 */
float dms_metrics_ear(const dms_point p[6])
{
    float vertical;
    float horizontal;

    vertical = dms_dist(&p[1], &p[5]) + dms_dist(&p[2], &p[4]);
    horizontal = dms_dist(&p[0], &p[3]);
    if (horizontal <= 0.0f) {
        return -1.0f;
    }
    return vertical / (2.0f * horizontal);
}

/* 输入：98 点关键点。输出：嘴部纵横比 MAR；唇宽为 0 时返回 -1。 */
float dms_metrics_mar(const dms_landmarks *lm)
{
    float mouth_height;
    float mouth_width;

    if (lm == NULL) {
        return -1.0f;
    }
    /* 98 点（WFLW）：嘴角 76（左）/ 82（右），内唇上中 90 / 下中 94。
     * 用内唇中点算唇高（比外唇更贴合嘴张开程度），嘴角距离算唇宽。
     * 注意：MAR 定义相对原 35 点模型变了，APP_DMS_MAR_YAWN 阈值可能需上板重标。 */
    mouth_height = dms_dist(&lm->pt[90], &lm->pt[94]);
    mouth_width = dms_dist(&lm->pt[76], &lm->pt[82]);
    if (mouth_width <= 0.0f) {
        return -1.0f;
    }
    return mouth_height / mouth_width;
}

/* 98 点眼组的 6 点 EAR 下标（左眼 68-75、右眼 60-67）。
 * 主机 onnxruntime 实测确认的眼组顺序：
 *   右眼 60-67 = 外角60 → 上睑61/62/63 → 内角64 → 下睑65/66/67；
 *   左眼 68-75 = 内角68 → 上睑69/70/71 → 外角72 → 下睑73/74/75。
 * dms_metrics_ear() 需要 p0..p5 = 眼角1、上睑1、上睑2、眼角2、下睑2、下睑1，
 * 且 p1/p5、p2/p4 必须是上下正对的两对点（同一竖线上）。
 * 98 点里下睑是从眼角2 往回排的：右眼 65 靠 64、67 靠 60；左眼 73 靠 72、75 靠 68。
 * 所以取上睑两端点 61/63（69/71）与正下方的 67/65（75/73）配对。
 * （2026-11-20 修正：原表取 {61,62}/{65,66}，配成了斜对角，EAR 被系统性放大，
 *   睁眼读数 0.4~0.54 偏高即由此而来。EAR 现已与分类器联合参与闭眼判定，见步骤 1。） */
static const int g_ear_idx[2][6] = {
    { 68, 69, 71, 72, 73, 75 },   /* 左眼：内角, 上睑(近内角), 上睑(近外角), 外角, 下睑(近外角), 下睑(近内角) */
    { 60, 61, 63, 64, 65, 67 },   /* 右眼：外角, 上睑(近外角), 上睑(近内角), 内角, 下睑(近内角), 下睑(近外角) */
};

void dms_metrics_ear_98(const dms_landmarks *lm, float *ear_left, float *ear_right)
{
    dms_point p[6];
    int side;
    int i;

    if (lm == NULL || ear_left == NULL || ear_right == NULL) {
        return;
    }
    for (side = 0; side < 2; side++) {
        for (i = 0; i < 6; i++) {
            p[i] = lm->pt[g_ear_idx[side][i]];
        }
        if (side == 0) {
            *ear_left = dms_metrics_ear(p);
        } else {
            *ear_right = dms_metrics_ear(p);
        }
    }
}

/* 输入：指标状态对象。输出：人脸跟丢时清掉进行中的“连续段”状态机和报警电平。
 * 注意：不清 PERCLOS 环形缓冲和哈欠时间戳（它们按时间窗自然过期）；
 * 只是避免人脸回来时拿很久以前的起点继续计时（例如长闭眼/分心被瞬间判满），
 * 以及跟丢期间报警电平一直停在旧值。 */
void dms_metrics_on_face_lost(dms_metrics *m)
{
    if (m == NULL) {
        return;
    }
    m->yawn_open = TD_FALSE;
    m->yawn_counted = TD_FALSE;
    m->blink_open = TD_FALSE;
    m->blink_counted = TD_FALSE;
    m->distract_open = TD_FALSE;
    m->distracted = TD_FALSE;
    m->prev_distracted = TD_FALSE;
}

/* 输入：指标状态对象。输出：清零状态并按 PERCLOS 窗口长度分配环形缓冲。 */
void dms_metrics_init(dms_metrics *m)
{
    td_u32 capacity;
    td_u32 win_seconds;

    if (m == NULL) {
        return;
    }
    memset(m, 0, sizeof(*m));
    m->ear_base = APP_DMS_EAR_OPEN;

    win_seconds = APP_DMS_PERCLOS_WIN_MS / 1000;
    if (win_seconds == 0) {
        win_seconds = 1;
    }
    capacity = (td_u32)APP_DMS_FPS * win_seconds + 8;
    m->ring.buf = (dms_sample *)calloc(capacity, sizeof(dms_sample));
    if (m->ring.buf == NULL) {
        printf("dms_metrics: ring buffer alloc failed (%u)\n", capacity);
        return;
    }
    m->ring.capacity = capacity;
    m->ring.head = 0;
    m->ring.count = 0;
}

/* 输入：指标状态对象。输出：释放环形缓冲并把容量清零。
 * 注意：缓冲只在 init 分配一次、这里释放一次，不是每帧 malloc/free。 */
void dms_metrics_deinit(dms_metrics *m)
{
    if (m == NULL) {
        return;
    }
    if (m->ring.buf != NULL) {
        free(m->ring.buf);
        m->ring.buf = NULL;
    }
    m->ring.capacity = 0;
    m->ring.head = 0;
    m->ring.count = 0;
}

/* 往环形缓冲写一个样本。 */
static void dms_ring_push(dms_ring *ring, const dms_sample *sample)
{
    if (ring->buf == NULL || ring->capacity == 0) {
        return;
    }
    ring->buf[ring->head] = *sample;
    ring->head = (ring->head + 1) % ring->capacity;
    if (ring->count < ring->capacity) {
        ring->count++;
    }
}

/* 统计窗口内的闭眼占比。返回 0~1；没有样本时返回 0。 */
static float dms_ring_perclos(const dms_ring *ring, double now_ms, double win_ms)
{
    td_u32 index;
    td_u32 checked = 0;
    td_u32 closed = 0;

    if (ring->buf == NULL || ring->count == 0) {
        return 0.0f;
    }
    /* 从最新往回遍历，最多 ring->count 个。 */
    for (index = 0; index < ring->count; index++) {
        td_u32 pos = (ring->head + ring->capacity - 1 - index) % ring->capacity;
        const dms_sample *s = &ring->buf[pos];

        if (now_ms - s->t_ms > win_ms) {
            break;
        }
        checked++;
        if (s->eye_closed != TD_FALSE) {
            closed++;
        }
    }
    if (checked == 0) {
        return 0.0f;
    }
    return (float)closed / (float)checked;
}

/* 清理窗口外的打哈欠记录，返回窗口内的次数。 */
static td_u32 dms_yawn_count_in_window(dms_metrics *m, double now_ms, double win_ms)
{
    td_u32 index;
    td_u32 count = 0;

    for (index = 0; index < m->yawn_count; index++) {
        if (now_ms - m->yawn_times[index] <= win_ms) {
            count++;
        }
    }
    return count;
}

static void dms_event_set(dms_event *ev, dms_event_type type, dms_alarm_level level,
                          double now_ms, const char *desc)
{
    if (ev == NULL) {
        return;
    }
    ev->type = type;
    ev->level = level;
    ev->t_ms = now_ms;
    snprintf(ev->desc, sizeof(ev->desc), "%s", desc);
}

/* ---------------- 分级报警 ---------------- */

/*
 * 每帧更新报警级别。
 *   eyes_off：本帧是否处于“视线脱离”（已确认的长闭眼 / 分心 / 长时间无人脸）。
 *   fatigue ：PERCLOS 超标或哈欠过多。
 *   chain_enabled：车速条件满足（或未接车速）。
 */
static void dms_alarm_update(dms_metrics *m, double now_ms, td_bool eyes_off, td_bool fatigue,
                             td_bool chain_enabled, dms_event *events, td_u32 *ev_count)
{
    dms_alarm_level eor_level = DMS_ALARM_NONE;
    dms_alarm_level fat_level = DMS_ALARM_NONE;

    /* 复位延时：视线回到路面要连续保持 APP_DMS_EOR_RELEASE_MS 才算真的回来。
     * 闭眼/分心检测偶尔断一两帧时链不复位，计时也不重来。 */
    if (eyes_off == TD_TRUE) {
        m->eor_latched = TD_TRUE;
        m->eor_on_since_ms = 0.0;
    } else if (m->eor_latched == TD_TRUE) {
        if (m->eor_on_since_ms <= 0.0) {
            m->eor_on_since_ms = now_ms;
        }
        if (now_ms - m->eor_on_since_ms >= (double)APP_DMS_EOR_RELEASE_MS) {
            m->eor_latched = TD_FALSE;
            m->eor_on_since_ms = 0.0;
        }
    }
    eyes_off = m->eor_latched;

    /* 视线脱离链：每进入一个阶段只发一次事件。 */
    if (eyes_off == TD_TRUE && chain_enabled == TD_TRUE) {
        double t;
        td_u32 stage;

        if (m->eor_open == TD_FALSE) {
            m->eor_open = TD_TRUE;
            m->eor_begin_ms = now_ms;
            m->eor_stage = 0;
        }
        t = now_ms - m->eor_begin_ms;
        stage = 1;
        if (t >= (double)APP_DMS_EOR_UPGRADE_MS) {
            stage = 2;
        }
        if (t >= (double)(APP_DMS_EOR_UPGRADE_MS + APP_DMS_EOR_DCA_MS)) {
            stage = 3;
        }
        if (t >= (double)(APP_DMS_EOR_UPGRADE_MS + APP_DMS_EOR_DCA_MS + APP_DMS_EOR_RMF_MS)) {
            stage = 4;
        }
        while (m->eor_stage < stage) {
            m->eor_stage++;
            switch (m->eor_stage) {
            case 1:
                if (events != NULL && *ev_count < DMS_MAX_EVENTS) {
                    dms_event_set(&events[(*ev_count)++], DMS_EVENT_EOR, DMS_ALARM_NOTICE,
                                  now_ms, "L1 EOR eyes off road");
                }
                break;
            case 2:
                if (events != NULL && *ev_count < DMS_MAX_EVENTS) {
                    dms_event_set(&events[(*ev_count)++], DMS_EVENT_EOR_UPGRADE, DMS_ALARM_WARNING,
                                  now_ms, "L2 upgraded EOR");
                }
                break;
            case 3:
                if (events != NULL && *ev_count < DMS_MAX_EVENTS) {
                    dms_event_set(&events[(*ev_count)++], DMS_EVENT_DCA, DMS_ALARM_STRONG,
                                  now_ms, "L3 DCA take over now");
                }
                break;
            default:
                if (events != NULL && *ev_count < DMS_MAX_EVENTS) {
                    dms_event_set(&events[(*ev_count)++], DMS_EVENT_RMF, DMS_ALARM_RMF,
                                  now_ms, "L4 RMF minimal risk maneuver");
                }
                break;
            }
        }
        eor_level = (dms_alarm_level)m->eor_stage;   /* 1..4 = NOTICE..RMF */
    } else {
        m->eor_open = TD_FALSE;
        m->eor_stage = 0;
    }

    /* 疲劳报警：条件成立期间 L1，持续 APP_DMS_ALARM_L2_MS 后 L2；不进入接管/RMF。 */
    if (fatigue == TD_TRUE) {
        if (m->fatigue_open == TD_FALSE) {
            m->fatigue_open = TD_TRUE;
            m->fatigue_begin_ms = now_ms;
        }
        fat_level = (now_ms - m->fatigue_begin_ms < (double)APP_DMS_ALARM_L2_MS) ?
                    DMS_ALARM_NOTICE : DMS_ALARM_WARNING;
    } else {
        m->fatigue_open = TD_FALSE;
    }

    m->level = (eor_level > fat_level) ? eor_level : fat_level;
}

static void dms_alarm_fill_status(const dms_metrics *m, dms_status *st)
{
    st->level = m->level;
    st->eor_stage = m->eor_stage;
}

void dms_metrics_process(dms_metrics *m, const dms_face_state *fs, double now_ms,
                         dms_status *st, dms_event *events, td_u32 *event_count)
{
    td_u32 ev_count = 0;
    float perclos;
    td_bool eye_closed;
    td_bool perclos_fatigue;
    td_bool yawn_fatigue;
    td_bool blink_event = TD_FALSE;
    td_bool blink_active = TD_FALSE;
    td_bool fatigue_now;

    if (m == NULL || fs == NULL || st == NULL || event_count == NULL) {
        return;
    }

    /*
     * 各状态变量分工：
     *   blink_open   眼睛当前是否处于“闭”这一段状态；
     *   blink_begin_ms 这一段闭眼何时开始；
     *   blink_counted 这一轮长闭眼事件是否已经发过，保证边沿只发一次；
     *   blink_active 闭眼持续超过阈值后的持续电平，用来撑住报警升级。
     */
    /* [步骤 0/9] 换人检测：连续无人脸太久，头姿基线和睁眼基线都重新学。 */
    if (m->last_face_ms > 0.0 &&
        now_ms - m->last_face_ms > (double)APP_DMS_POSE_RECALIB_LOST_MS) {
        m->pose_base_n = 0;
        m->ear_base_n = 0;
        m->ear_base = APP_DMS_EAR_OPEN;
        m->first_sample_ms = 0.0;
    }
    if (m->first_sample_ms <= 0.0) {
        m->first_sample_ms = now_ms;
    }
    m->last_face_ms = now_ms;

    /* [步骤 1/9] 闭眼判定：分类器两眼都判闭 且 EAR 明显低于本人睁眼基线。
     * 只用分类器：戴眼镜反光时常把睁眼判闭；只用 EAR：PFLD 闭眼时眼睑点不完全合拢，
     * 睁闭差距小。两者“且”起来才稳（标定依据见 app_config.h）。
     * EAR 不可用时退回只看分类器。 */
    {
        td_bool cls_closed = (fs->eye_left == DMS_EYE_CLOSED &&
                              fs->eye_right == DMS_EYE_CLOSED) ? TD_TRUE : TD_FALSE;
        td_bool cls_open = (fs->eye_left == DMS_EYE_OPEN &&
                            fs->eye_right == DMS_EYE_OPEN) ? TD_TRUE : TD_FALSE;

        eye_closed = TD_FALSE;
        if (fs->ear >= 0.0f) {
            eye_closed = (cls_closed == TD_TRUE &&
                          fs->ear < m->ear_base * APP_DMS_EAR_CLOSED_RATIO) ? TD_TRUE : TD_FALSE;
            /* 睁眼基线：分类器判睁的帧才参与。
             * 前 24 帧收集后取中位数（转头/侧脸时 EAR 会冲到 0.4 以上，平均值会被带偏，
             * 板端实测基线被拉到 0.42，导致正常睁眼也被判闭）；之后用慢速指数平均跟随，
             * 每个样本先钳到基线的 0.8~1.2 倍，单帧离群值拉不动基线。 */
            if (cls_open == TD_TRUE && fs->ear > 0.05f && fs->ear < 0.6f) {
                if (m->ear_base_n < 24) {
                    m->ear_init[m->ear_base_n++] = fs->ear;
                    if (m->ear_base_n == 24) {
                        float tmp[24];
                        int a;
                        int b;
                        memcpy(tmp, m->ear_init, sizeof(tmp));
                        for (a = 1; a < 24; a++) {          /* 插入排序，24 个数够快 */
                            float v = tmp[a];
                            for (b = a - 1; b >= 0 && tmp[b] > v; b--) {
                                tmp[b + 1] = tmp[b];
                            }
                            tmp[b + 1] = v;
                        }
                        m->ear_base = 0.5f * (tmp[11] + tmp[12]);
                    }
                } else {
                    float e = fs->ear;
                    if (e > m->ear_base * 1.2f) { e = m->ear_base * 1.2f; }
                    if (e < m->ear_base * 0.8f) { e = m->ear_base * 0.8f; }
                    m->ear_base += (e - m->ear_base) / (10.0f * (float)APP_DMS_FPS);
                }
            }
        } else {
            eye_closed = cls_closed;
        }
    }

    /* [步骤 2/9] 打哈欠状态机。MAR 连续超过阈值到保持时长，才记一次哈欠。
     * 头姿基线标定好后，转头超过 APP_DMS_YAWN_MAX_YAW 不计（侧脸嘴宽变窄，MAR 虚高）。 */
    {
        td_bool facing = TD_TRUE;

        if (fs->pose_valid == TD_TRUE && m->pose_base_n >= (td_u32)APP_DMS_POSE_CALIB_FRAMES &&
            fabsf(fs->yaw - m->pose_base_yaw) > APP_DMS_YAWN_MAX_YAW) {
            facing = TD_FALSE;
        }
        if (facing == TD_TRUE && fs->mar >= 0.0f && fs->mar > APP_DMS_MAR_YAWN) {
            m->yawn_last_high_ms = now_ms;
            if (m->yawn_open == TD_FALSE) {
                m->yawn_open = TD_TRUE;
                m->yawn_counted = TD_FALSE;
                m->yawn_begin_ms = now_ms;
            }
            if (m->yawn_counted == TD_FALSE &&
                now_ms - m->yawn_begin_ms >= (double)APP_DMS_YAWN_HOLD_MS) {
                td_u32 keep = 0;
                td_u32 index;

                for (index = 0; index < m->yawn_count; index++) {
                    if (now_ms - m->yawn_times[index] <= (double)APP_DMS_YAWN_WIN_MS) {
                        m->yawn_times[keep++] = m->yawn_times[index];
                    }
                }
                m->yawn_count = keep;
                if (m->yawn_count < DMS_YAWN_MAX) {
                    m->yawn_times[m->yawn_count] = now_ms;
                    m->yawn_count++;
                }
                m->yawn_counted = TD_TRUE;
            }
        } else if (m->yawn_open == TD_TRUE && now_ms - m->yawn_last_high_ms > 300.0) {
            /* 去抖：哈欠中间 MAR 偶尔一帧掉到门槛下不算闭嘴，低于门槛超过 300ms 才结束。 */
            m->yawn_open = TD_FALSE;
        }
    }

    /* [步骤 3/9] 长闭眼状态机。blink_event 是只发一次的边沿，blink_active 是持续电平。 */
    /* 去抖：闭眼段中间偶尔一帧判睁（分类器抖动）不算睁眼。要连续 2 帧判睁、且距最后
     * 一次判闭超过 300ms 才结束这一段闭眼（帧率低时 300ms 可能不到一帧，所以两条都要）。 */
    if (eye_closed == TD_TRUE) {
        m->blink_last_closed_ms = now_ms;
        m->blink_open_frames = 0;
        if (m->blink_open == TD_FALSE) {
            m->blink_open = TD_TRUE;
            m->blink_counted = TD_FALSE;
            m->blink_begin_ms = now_ms;
        }
    } else if (m->blink_open == TD_TRUE) {
        m->blink_open_frames++;
        if (m->blink_open_frames >= 2 && now_ms - m->blink_last_closed_ms > 300.0) {
            m->blink_open = TD_FALSE;
        }
    }
    if (m->blink_open == TD_TRUE && m->blink_counted == TD_FALSE &&
        now_ms - m->blink_begin_ms >= (double)APP_DMS_LONG_BLINK_MS) {
        blink_event = TD_TRUE;
        m->blink_counted = TD_TRUE;
    }
    blink_active = (m->blink_open == TD_TRUE &&
                    now_ms - m->blink_begin_ms >= (double)APP_DMS_LONG_BLINK_MS)
                   ? TD_TRUE : TD_FALSE;

    /* [步骤 4/9] 分心状态机：头姿相对本人正常姿态基线越界，并连续保持一段时间。
     * 基线：开机/换人后前 APP_DMS_POSE_CALIB_FRAMES 帧有效头姿取平均（期间不判分心），
     * 之后只在未越界时按 APP_DMS_POSE_ADAPT_SECONDS 时间常数缓慢跟随坐姿变化。 */
    {
        float yaw_rel = 0.0f;
        float pitch_rel = 0.0f;
        td_bool pose_out = TD_FALSE;

        if (fs->pose_valid == TD_TRUE) {
            if (m->pose_base_n < (td_u32)APP_DMS_POSE_CALIB_FRAMES) {
                m->pose_base_yaw = (m->pose_base_yaw * (float)m->pose_base_n + fs->yaw) /
                                   (float)(m->pose_base_n + 1);
                m->pose_base_pitch = (m->pose_base_pitch * (float)m->pose_base_n + fs->pitch) /
                                     (float)(m->pose_base_n + 1);
                m->pose_base_n++;
            } else {
                yaw_rel = fs->yaw - m->pose_base_yaw;
                pitch_rel = fs->pitch - m->pose_base_pitch;
                if (m->distract_open == TD_TRUE) {
                    /* 已越界：要回到门槛内侧 APP_DMS_DISTRACT_EXIT_MARGIN 度才算回正（回差）。 */
                    pose_out = (fabsf(yaw_rel) > APP_DMS_YAW_THRESH - APP_DMS_DISTRACT_EXIT_MARGIN ||
                                pitch_rel < APP_DMS_PITCH_THRESH + APP_DMS_DISTRACT_EXIT_MARGIN)
                               ? TD_TRUE : TD_FALSE;
                } else {
                    pose_out = (fabsf(yaw_rel) > APP_DMS_YAW_THRESH ||
                                pitch_rel < APP_DMS_PITCH_THRESH) ? TD_TRUE : TD_FALSE;
                }
                if (pose_out == TD_FALSE) {
                    float k = 1.0f / ((float)APP_DMS_POSE_ADAPT_SECONDS * (float)APP_DMS_FPS);
                    m->pose_base_yaw += yaw_rel * k;
                    m->pose_base_pitch += pitch_rel * k;
                }
            }
        }
        st->yaw_rel = yaw_rel;
        st->pitch_rel = pitch_rel;
        st->pose_cal = (m->pose_base_n >= (td_u32)APP_DMS_POSE_CALIB_FRAMES) ? TD_TRUE : TD_FALSE;

        if (pose_out == TD_TRUE) {
            if (m->distract_open == TD_FALSE) {
                m->distract_open = TD_TRUE;
                m->distract_begin_ms = now_ms;
                m->distracted = TD_FALSE;
            }
            if (m->distracted == TD_FALSE &&
                now_ms - m->distract_begin_ms >= (double)APP_DMS_DISTRACT_HOLD_MS) {
                m->distracted = TD_TRUE;
            }
        } else {
            m->distract_open = TD_FALSE;
            m->distracted = TD_FALSE;
        }
    }

    /* [步骤 5/9] PERCLOS。同时算哈欠次数，得到一个总的“当前是否疲劳”布尔值。 */
    perclos = dms_ring_perclos(&m->ring, now_ms, (double)APP_DMS_PERCLOS_WIN_MS);
    /* 数据不满半个窗口（刚开机/换人）时样本太少，一次眨眼就能把占比顶过阈值，不判。
     * 带滞回：超过阈值进入疲劳，降到阈值 0.8 倍以下才退出，避免在阈值附近反复触发事件。 */
    if (now_ms - m->first_sample_ms < (double)APP_DMS_PERCLOS_WIN_MS * 0.5) {
        perclos_fatigue = TD_FALSE;
    } else if (m->prev_perclos_fatigue == TD_TRUE) {
        perclos_fatigue = (perclos > APP_DMS_PERCLOS_THRESH * 0.8f) ? TD_TRUE : TD_FALSE;
    } else {
        perclos_fatigue = (perclos > APP_DMS_PERCLOS_THRESH) ? TD_TRUE : TD_FALSE;
    }
    yawn_fatigue = (dms_yawn_count_in_window(m, now_ms, (double)APP_DMS_YAWN_WIN_MS)
                    >= APP_DMS_YAWN_COUNT) ? TD_TRUE : TD_FALSE;
    /* PERCLOS / 哈欠按 60 秒窗口统计：条件成立期间一直报警，窗口里不再满足才解除。 */
    fatigue_now = perclos_fatigue || yawn_fatigue;

    /* [步骤 6/9] 写环形缓冲（只记有效人脸且眼睛分类有结果的样本，
     * 眼睛 UNKNOWN 的帧既不算睁也不算闭，不能稀释 PERCLOS）。 */
    if (fs->eye != DMS_EYE_UNKNOWN || eye_closed == TD_TRUE) {
        dms_sample sample;
        sample.t_ms = now_ms;
        sample.eye_closed = eye_closed;
        sample.yawn = m->yawn_open;
        sample.distracted = m->distracted;
        /* 记录当时的 EAR，便于事后分析（闭眼判定见步骤 1）。 */
        sample.ear = fs->ear;
        dms_ring_push(&m->ring, &sample);
    }

    /* [步骤 7/9] 报警状态机：视线脱离链和疲劳报警。 */
    m->noface_since_ms = 0.0;
    m->face_seen = TD_TRUE;
    dms_alarm_update(m, now_ms,
                     (m->distracted == TD_TRUE || blink_active == TD_TRUE) ? TD_TRUE : TD_FALSE,
                     fatigue_now,
                     (fs->speed_kmh < 0.0f || fs->speed_kmh > APP_DMS_MIN_SPEED_KMH) ? TD_TRUE : TD_FALSE,
                     events, &ev_count);

    /* [步骤 8/9] 边沿事件。只在条件刚变成真时发一次，避免每帧重复触发录像。 */
    /* 长闭眼只发一次；PERCLOS 和哈欠只在 false→true 时发；分心由独立 prev 状态标记。 */
    if (events != NULL) {
        if (blink_event == TD_TRUE && ev_count < DMS_MAX_EVENTS) {
            dms_event_set(&events[ev_count++], DMS_EVENT_FATIGUE_BLINK, m->level,
                          now_ms, "long eye closure");
        }
        if (perclos_fatigue == TD_TRUE && m->prev_perclos_fatigue == TD_FALSE && ev_count < DMS_MAX_EVENTS) {
            dms_event_set(&events[ev_count++], DMS_EVENT_FATIGUE_PERCLOS, m->level,
                          now_ms, "PERCLOS over threshold");
        }
        if (yawn_fatigue == TD_TRUE && m->prev_yawn_fatigue == TD_FALSE && ev_count < DMS_MAX_EVENTS) {
            dms_event_set(&events[ev_count++], DMS_EVENT_YAWN, m->level,
                          now_ms, "yawning too often");
        }
        if (m->distracted == TD_TRUE && m->distract_open == TD_TRUE && ev_count < DMS_MAX_EVENTS) {
            /* 只在刚进入分心时发一次：用 distracted 的边沿。 */
            if (m->prev_distracted == TD_FALSE) {
                dms_event_set(&events[ev_count++], DMS_EVENT_DISTRACT, m->level,
                              now_ms, "head pose distraction");
            }
            m->prev_distracted = TD_TRUE;
        }
    }
    m->prev_perclos_fatigue = perclos_fatigue;
    m->prev_yawn_fatigue = yawn_fatigue;
    m->prev_distracted = m->distracted;

    /* [步骤 9/9] 状态快照。把所有显示/日志要用的值打包成快照交给调用方。 */
    st->t_ms = now_ms;
    st->ear = fs->ear;   /* 双眼平均 EAR，透传给日志/叠加 */
    st->mar = fs->mar;
    st->perclos = perclos;
    /* 显示的眼睛状态 = 最终判定结果（不是分类器原始结果）。 */
    if (eye_closed == TD_TRUE) {
        st->eye = DMS_EYE_CLOSED;
    } else if (fs->eye == DMS_EYE_UNKNOWN) {
        st->eye = DMS_EYE_UNKNOWN;
    } else {
        st->eye = DMS_EYE_OPEN;
    }
    st->ear_base = m->ear_base;
    st->yaw = fs->yaw;
    st->pitch = fs->pitch;
    st->roll = fs->roll;
    st->yawn_count = dms_yawn_count_in_window(m, now_ms, (double)APP_DMS_YAWN_WIN_MS);
    st->distracted = m->distracted;
    st->perclos_fatigue = perclos_fatigue;
    st->yawn_fatigue = yawn_fatigue;
    st->fatigue_alert = fatigue_now;
    st->long_closure = blink_active;
    st->face_count = 1;
    dms_alarm_fill_status(m, st);
    st->face_present = TD_TRUE;

    *event_count = ev_count;
}

void dms_metrics_process_noface(dms_metrics *m, double now_ms, dms_status *st,
                                dms_event *events, td_u32 *event_count)
{
    td_u32 ev_count = 0;
    td_bool eyes_off = TD_FALSE;
    float perclos;

    if (m == NULL || st == NULL || event_count == NULL) {
        return;
    }
    memset(st, 0, sizeof(*st));
    /* 曾有人脸、之后连续无人脸 ≥ 分心保持时间：驾驶员可能大幅转头/离座，按视线脱离处理。
     * 开机就没人（face_seen 为假）不算，避免空座位一直报警。 */
    if (m->noface_since_ms <= 0.0) {
        m->noface_since_ms = now_ms;
    }
    if (m->face_seen == TD_TRUE &&
        now_ms - m->noface_since_ms >= (double)APP_DMS_DISTRACT_HOLD_MS) {
        eyes_off = TD_TRUE;
    }
    /* 疲劳统计在无人脸期间不更新，沿用窗口里已有的结论。 */
    perclos = dms_ring_perclos(&m->ring, now_ms, (double)APP_DMS_PERCLOS_WIN_MS);
    dms_alarm_update(m, now_ms, eyes_off,
                     (m->prev_perclos_fatigue == TD_TRUE ||
                      dms_yawn_count_in_window(m, now_ms, (double)APP_DMS_YAWN_WIN_MS) >=
                      APP_DMS_YAWN_COUNT) ? TD_TRUE : TD_FALSE,
                     TD_TRUE, events, &ev_count);
    st->t_ms = now_ms;
    st->ear = -1.0f;
    st->mar = -1.0f;
    st->perclos = perclos;
    st->eye = DMS_EYE_UNKNOWN;
    st->ear_base = m->ear_base;
    st->yawn_count = dms_yawn_count_in_window(m, now_ms, (double)APP_DMS_YAWN_WIN_MS);
    st->perclos_fatigue = m->prev_perclos_fatigue;
    st->fatigue_alert = (m->fatigue_open == TD_TRUE) ? TD_TRUE : TD_FALSE;
    st->distracted = eyes_off;
    st->face_count = 0;
    st->face_present = TD_FALSE;
    dms_alarm_fill_status(m, st);
    *event_count = ev_count;
}
