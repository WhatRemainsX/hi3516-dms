/*
 * DMS 后台线程由 main.c 启动和停止，从 VPSS 通道 2 取未编码的图像帧。
 *
 * dms_models 提取人脸、关键点、头姿和眼睛分类结果。dms_detect 负责
 * 检测框的解码和跟踪。单帧结果存入 dms_face_state，再交给 dms_metrics
 * 更新跨帧状态，得到当前 dms_status 和本帧新产生的 dms_event。
 *
 * 当前状态用于声光提示、日志和画面叠加。新事件交给 dms_sink，后者记录
 * 事件日志并请求 dms_clip 录制。事件录像的 H.264 来自 media_source。
 * dms_thread_main 负责向 VPSS 借帧和还帧，dms_process_frame 只处理借来的图像。
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>

#include "app_config.h"
#include "dms_thread.h"
#include "dms_models.h"
#include "dms_detect.h"
#include "dms_metrics.h"
#include "dms_sink.h"
#include "dms_log.h"
#include "dms_clip.h"
#include "dms_occlusion.h"
#include "dms_alert.h"
#include "npu_model.h"
#include "ss_mpi_vpss.h"

#define DMS_GET_FRAME_TIMEOUT_MS 1000
/* 没有新检测结果的帧数超过此值时，丢弃跟踪框并进入无人脸处理。 */
#define DMS_MAX_MISSED 10
/* 遮挡自检的间隔帧数。 */
#define DMS_OCCLUSION_INTERVAL 30

static pthread_t g_thread;
static td_bool g_created = TD_FALSE;
static atomic_bool g_stop = ATOMIC_VAR_INIT(false);

static dms_models g_models;
static dms_tracker g_tracker;
static dms_metrics g_metrics;
static td_u32 g_frames = 0;
static double g_last_log_ms = 0.0;
/* 上一帧是否有有效人脸；用于在“有脸→没脸”边沿清一次状态机。 */
static td_bool g_face_present = TD_FALSE;

/* 输入：无。输出：单调时钟毫秒数，不受系统改时间影响。 */
static double dms_now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1000000.0;
}

#if APP_DMS_PROFILE
/*
 * 性能统计（只在 APP_DMS_PROFILE=1 时编译）：
 *   每 5 秒打印一行 dms_prof：实际帧率、各步骤平均耗时（毫秒）、DMS 线程忙碌比例、
 *   本进程和整机 CPU 占用。det 是"跑了检测的那些帧"的平均值，其余是每帧平均。
 */
#include <sys/syscall.h>
#include <unistd.h>

enum { PF_WAIT, PF_DET, PF_LM, PF_POSE, PF_EYE, PF_TOTAL, PF_NUM };
static double g_pf_sum[PF_NUM];
static td_u32 g_pf_frames;
static td_u32 g_pf_det_runs;
static double g_pf_begin_ms;
static unsigned long long g_pf_proc_ticks;
static unsigned long long g_pf_sys_busy;
static unsigned long long g_pf_sys_total;

/* 读本进程累计 CPU 时间（用户态 + 内核态，单位 tick）。 */
static unsigned long long dms_pf_proc_ticks(void)
{
    FILE *fp = fopen("/proc/self/stat", "r");
    char buf[512];
    char *p;
    unsigned long long ut = 0;
    unsigned long long stt = 0;

    if (fp == NULL) {
        return 0;
    }
    if (fgets(buf, sizeof(buf), fp) == NULL) {
        fclose(fp);
        return 0;
    }
    fclose(fp);
    p = strrchr(buf, ')');          /* 进程名里可能有空格，从最后一个 ')' 之后开始数字段 */
    if (p == NULL) {
        return 0;
    }
    /* ')' 之后依次是 state(3) ppid ... utime(14) stime(15) */
    if (sscanf(p + 2, "%*c %*d %*d %*d %*d %*d %*u %*u %*u %*u %*u %llu %llu", &ut, &stt) != 2) {
        return 0;
    }
    return ut + stt;
}

/* 读整机 CPU 累计时间：busy = 非空闲 tick，total = 全部 tick。 */
static void dms_pf_sys_ticks(unsigned long long *busy, unsigned long long *total)
{
    FILE *fp = fopen("/proc/stat", "r");
    unsigned long long v[8] = {0};

    *busy = 0;
    *total = 0;
    if (fp == NULL) {
        return;
    }
    if (fscanf(fp, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]) == 8) {
        *total = v[0] + v[1] + v[2] + v[3] + v[4] + v[5] + v[6] + v[7];
        *busy = *total - v[3] - v[4];   /* 减去 idle 和 iowait */
    }
    fclose(fp);
}

static void dms_pf_add(int idx, double ms)
{
    g_pf_sum[idx] += ms;
}

/* 每帧结束时调用；满 5 秒打印一行并清零。 */
static void dms_pf_tick(double now)
{
    double wall;
    unsigned long long proc;
    unsigned long long busy;
    unsigned long long total;
    long hz = sysconf(_SC_CLK_TCK);
    double n;

    g_pf_frames++;
    if (g_pf_begin_ms <= 0.0) {
        g_pf_begin_ms = now;
        g_pf_proc_ticks = dms_pf_proc_ticks();
        dms_pf_sys_ticks(&g_pf_sys_busy, &g_pf_sys_total);
        memset(g_pf_sum, 0, sizeof(g_pf_sum));
        g_pf_frames = 0;
        g_pf_det_runs = 0;
        return;
    }
    wall = now - g_pf_begin_ms;
    if (wall < 5000.0 || g_pf_frames == 0) {
        return;
    }
    proc = dms_pf_proc_ticks();
    dms_pf_sys_ticks(&busy, &total);
    n = (double)g_pf_frames;
    printf("dms_prof: fps=%.1f det=%.1fms(x%u) lm=%.1f pose=%.1f eye=%.1f rest=%.1f "
           "total=%.1fms wait=%.1fms busy=%.0f%% proc_cpu=%.0f%% sys_cpu=%.0f%%\n",
           n * 1000.0 / wall,
           g_pf_det_runs > 0 ? g_pf_sum[PF_DET] / (double)g_pf_det_runs : 0.0, g_pf_det_runs,
           g_pf_sum[PF_LM] / n, g_pf_sum[PF_POSE] / n, g_pf_sum[PF_EYE] / n,
           (g_pf_sum[PF_TOTAL] - g_pf_sum[PF_DET] - g_pf_sum[PF_LM] - g_pf_sum[PF_POSE] -
            g_pf_sum[PF_EYE]) / n, g_pf_sum[PF_TOTAL] / n, g_pf_sum[PF_WAIT] / n,
           100.0 * g_pf_sum[PF_TOTAL] / wall,
           hz > 0 ? 100.0 * (double)(proc - g_pf_proc_ticks) / ((double)hz * wall / 1000.0) : 0.0,
           total > g_pf_sys_total ? 100.0 * (double)(busy - g_pf_sys_busy) /
                                    (double)(total - g_pf_sys_total) : 0.0);
    g_pf_begin_ms = now;
    g_pf_proc_ticks = proc;
    g_pf_sys_busy = busy;
    g_pf_sys_total = total;
    memset(g_pf_sum, 0, sizeof(g_pf_sum));
    g_pf_frames = 0;
    g_pf_det_runs = 0;
}
#define PF_MARK(var) double var = dms_now_ms()
#define PF_ADD(idx, since) dms_pf_add((idx), dms_now_ms() - (since))
#define PF_DET_RUN() (g_pf_det_runs++)
#define PF_TICK() dms_pf_tick(dms_now_ms())
#else
#define PF_MARK(var) do { } while (0)
#define PF_ADD(idx, since) do { } while (0)
#define PF_DET_RUN() do { } while (0)
#define PF_TICK() do { } while (0)
#endif



/*
 * LED 跟随报警级别。蜂鸣器在长闭眼或视线脱离达到 L3/L4 时持续响，
 * 其余疲劳或视线脱离报警时断续响。报警解除后关闭蜂鸣器。
 */
static void dms_apply_alert(const dms_status *st)
{
    dms_alert_set_level(st->level);
    if (st->long_closure == TD_TRUE || st->eor_stage >= 3) {
        dms_alert_set_buzzer(DMS_BUZZER_CONTINUOUS);
    } else if (st->fatigue_alert == TD_TRUE || st->eor_stage >= 1) {
        dms_alert_set_buzzer(DMS_BUZZER_INTERMITTENT);
    } else {
        dms_alert_set_buzzer(DMS_BUZZER_OFF);
    }
}

/*
 * 跟踪刚丢失时清理上次的头姿初值和长闭眼、哈欠、分心的连续计时。
 * 无人脸期间仍运行报警状态机。曾看到人脸，随后持续无人脸 3.5 秒时，
 * 按视线脱离处理。
 */
static void dms_report_no_face(double now)
{
    dms_status st;
    dms_event events[DMS_MAX_EVENTS];
    td_u32 event_count = 0;
    td_u32 index;

    if (g_face_present == TD_TRUE) {
        dms_models_reset_pose();
        dms_metrics_on_face_lost(&g_metrics);
        g_face_present = TD_FALSE;
    }
    dms_metrics_process_noface(&g_metrics, now, &st, events, &event_count);
    for (index = 0; index < event_count; index++) {
        dms_sink_notify_event(&events[index]);
    }
    dms_apply_alert(&st);
    if (now - g_last_log_ms >= (double)APP_DMS_LOG_INTERVAL_MS) {
        char text[DMS_OVERLAY_TEXT_MAX];

        printf("dms: %.1fs level=%s no face eor=%u\n", now / 1000.0,
               dms_level_text(st.level), st.eor_stage);
        snprintf(text, sizeof(text), "DMS L%d NOFACE", (int)st.level);
        dms_sink_overlay(text);
        g_last_log_ms = now;
    }
}

/*
 * 处理一帧借来的 VPSS 图像。now 是本帧判定使用的单调时钟，单位毫秒。
 * 每次返回后，都由外层循环统一归还图像。
 */
static void dms_process_frame(const ot_video_frame *frame)
{
    dms_box det[DMS_MAX_FACES];
    td_u32 det_count = 0;
    td_u32 index;
    td_u32 event_count = 0;
    dms_face_state fs;
    dms_status st;
    dms_event events[DMS_MAX_EVENTS];
    char text[DMS_OVERLAY_TEXT_MAX];
    double now;
    td_s32 ret;

    now = dms_now_ms();
    g_frames++;

    /* 1. 周期性运行 SCRFD。两次检测之间沿用跟踪框，减少 NPU 推理次数。 */
    if ((g_frames % APP_DMS_DETECT_EVERY) == 1) {
        PF_MARK(pf_det);
        ret = dms_models_detect(&g_models, frame, det, DMS_MAX_FACES, &det_count);
        PF_ADD(PF_DET, pf_det);
        PF_DET_RUN();
        if (ret == TD_SUCCESS) {
            dms_tracker_on_detect(&g_tracker, det, det_count,
                                  APP_DMS_TRACK_IOU, APP_DMS_BOX_SMOOTH);
        } else {
            dms_tracker_on_skip(&g_tracker);
        }
    } else {
        dms_tracker_on_skip(&g_tracker);
    }

    if (g_tracker.valid == TD_TRUE && g_tracker.missed > DMS_MAX_MISSED) {
        /* 丢弃过期框，避免后续重新检测到人脸时拿旧框做匹配和平滑。 */
        dms_tracker_init(&g_tracker);
    }
    if (g_tracker.valid == TD_FALSE) {
        dms_report_no_face(now);
        return;
    }

    /* 2. 生成单帧识别结果。人脸框和关键点都用同一个 640x640 坐标系。 */
    memset(&fs, 0, sizeof(fs));
    dms_detect_expand(&g_tracker.box, APP_DMS_BOX_EXPAND,
                      (float)APP_DMS_IN_WIDTH, (float)APP_DMS_IN_HEIGHT, &fs.box);

    /*
     * fs.box 是外扩后用来记录的框。PFLD 用原始跟踪框，自行外扩 1.1 倍并裁成
     * 112x112 输入。再套一层外扩会缩小模型输入里的人脸。
     */
    PF_MARK(pf_lm);
    ret = dms_models_run_landmarks(&g_models, frame, &g_tracker.box, &fs.lm);
    PF_ADD(PF_LM, pf_lm);
    if (ret != TD_SUCCESS) {
        return;
    }
    fs.lm_valid = TD_TRUE;
    g_face_present = TD_TRUE;

    /* 从 98 个关键点中取 6 个点做 PnP 解算。失败时保留 pose_valid = FALSE。 */
    PF_MARK(pf_pose);
    ret = dms_models_run_pose(&fs.lm, &fs.yaw, &fs.pitch, &fs.roll, NULL);
    PF_ADD(PF_POSE, pf_pose);
    if (ret == TD_SUCCESS) {
        fs.pose_valid = TD_TRUE;
    }

    /* 两眼各裁一次 32x32 图：用关键点定眼框、按 roll 旋正，避免斜脸把眼睛裁歪。
     * PnP 失败时 fs.roll 为 0，歪头时眼框就不转正了；这时改用两外眼角连线
     * （60 画面左、72 画面右）估 roll，符号与 PnP 一致（画面顺时针为正）。 */
    {
        float eye_roll = fs.roll;

        if (fs.pose_valid == TD_FALSE) {
            float ex = fs.lm.pt[72].x - fs.lm.pt[60].x;
            float ey = fs.lm.pt[72].y - fs.lm.pt[60].y;
            if (ex > 1.0f) {
                eye_roll = atan2f(ey, ex) * 180.0f / 3.14159265358979323846f;
            }
        }
        PF_MARK(pf_eye);
        ret = dms_models_run_eye(&g_models, frame, &fs.lm, eye_roll,
                                 &fs.eye_left, &fs.eye_right, NULL);
        PF_ADD(PF_EYE, pf_eye);
    }
    if (ret == TD_SUCCESS) {
        /* fs.eye 汇总分类器是否有结果。最终闭眼规则使用左右眼各自的结果和 EAR。 */
        if (fs.eye_left == DMS_EYE_CLOSED || fs.eye_right == DMS_EYE_CLOSED) {
            fs.eye = DMS_EYE_CLOSED;
        } else if (fs.eye_left == DMS_EYE_OPEN && fs.eye_right == DMS_EYE_OPEN) {
            fs.eye = DMS_EYE_OPEN;
        } else {
            fs.eye = DMS_EYE_UNKNOWN;
        }
    }

    fs.mar = dms_metrics_mar(&fs.lm);

    /* EAR 是眼睛纵横比。有效时取双眼平均，只剩一只有效眼时使用该眼。 */
    dms_metrics_ear_98(&fs.lm, &fs.ear_left, &fs.ear_right);
    if (fs.ear_left >= 0.0f && fs.ear_right >= 0.0f) {
        fs.ear = (fs.ear_left + fs.ear_right) * 0.5f;
    } else if (fs.ear_left >= 0.0f) {
        fs.ear = fs.ear_left;
    } else if (fs.ear_right >= 0.0f) {
        fs.ear = fs.ear_right;
    } else {
        fs.ear = -1.0f;
    }
    fs.ear_valid = TD_FALSE;   /* 旧字段。当前用 fs.ear < 0 表示 EAR 无效。 */
    fs.speed_kmh = dms_sink_get_speed_kmh();   /* -1 表示车速未知，允许视线脱离报警。 */

    /*
     * 3. 更新指标和报警。st 是当前结果，每帧都用它刷新声光报警。
     * events 记录新发生的状态变化，逐条发送给日志和事件录像模块。
     */
    dms_metrics_process(&g_metrics, &fs, now, &st, events, &event_count);
    dms_apply_alert(&st);
    for (index = 0; index < event_count; index++) {
        dms_sink_notify_event(&events[index]);
    }

    /* 4. 按配置的周期刷新状态日志和 OSD（画面上的状态文字）。 */
    if (now - g_last_log_ms >= (double)APP_DMS_LOG_INTERVAL_MS) {
        dms_log_status(&st, fs.speed_kmh);
        /*
         * L0~L4 是报警级别，EYE 使用状态机给出的闭眼结论。
         * YAW/PITCH 显示相对个人基线的偏差。标定完成前显示 POSE CAL。
         */
        if (st.pose_cal == TD_TRUE) {
            snprintf(text, sizeof(text),
                     "DMS L%d PERCLOS %.2f MAR %.2f EAR %.2f EYE %s YAW %.0f PITCH %.0f YAWN %u",
                     (int)st.level, st.perclos, st.mar,
                     st.ear,
                     st.eye == DMS_EYE_OPEN ? "OPEN" :
                     (st.eye == DMS_EYE_CLOSED ? "CLOSED" : "UNK"),
                     st.yaw_rel, st.pitch_rel, st.yawn_count);
        } else {
            snprintf(text, sizeof(text),
                     "DMS L%d PERCLOS %.2f MAR %.2f EAR %.2f EYE %s POSE CAL YAWN %u",
                     (int)st.level, st.perclos, st.mar,
                     st.ear,
                     st.eye == DMS_EYE_OPEN ? "OPEN" :
                     (st.eye == DMS_EYE_CLOSED ? "CLOSED" : "UNK"),
                     st.yawn_count);
        }
        dms_sink_overlay(text);
        g_last_log_ms = now;
    }

    /* 5. 间隔检查画面亮度方差。目前遮挡检查只输出日志。 */
    if ((g_frames % DMS_OCCLUSION_INTERVAL) == 0) {
        dms_occlusion_check(frame);
    }
}

static void *dms_thread_main(void *arg)
{
    td_s32 ret;

    (void)arg;

    ret = npu_model_thread_attach();
    if (ret != TD_SUCCESS) {
        printf("dms_thread: npu attach failed, DMS disabled\n");
        return NULL;
    }

    g_last_log_ms = dms_now_ms();
    printf("dms_thread started\n");
    /* 获取成功后由本循环归还一次，保证 VPSS 可以复用图像缓冲区。 */
    while (atomic_load(&g_stop) == TD_FALSE) {
        ot_video_frame_info frame_info;
        PF_MARK(pf_wait);

        ret = ss_mpi_vpss_get_chn_frame(0, APP_DMS_VPSS_CHN, &frame_info,
                                        DMS_GET_FRAME_TIMEOUT_MS);
        if (ret != TD_SUCCESS) {
            continue;   /* 取帧失败，手上没有可归还的帧。 */
        }
        PF_ADD(PF_WAIT, pf_wait);

        {
            PF_MARK(pf_total);
            dms_process_frame(&frame_info.video_frame);
            PF_ADD(PF_TOTAL, pf_total);
        }

        /* 单帧处理提前返回时也走到这里。 */
        ret = ss_mpi_vpss_release_chn_frame(0, APP_DMS_VPSS_CHN, &frame_info);
        if (ret != TD_SUCCESS) {
            printf("dms_thread: release frame failed: %#x\n", (td_u32)ret);
        }
        PF_TICK();
    }

    printf("dms_thread stopped, processed %u frame(s)\n", g_frames);
    return NULL;
}

/*
 * 初始化输出接口、NPU、模型和判定状态，最后创建 DMS 线程。
 * TD_SUCCESS 表示线程已创建。线程运行后会尝试绑定它自己的 NPU 上下文。
 */
td_s32 dms_thread_start(void)
{
    int thread_ret;

#if !APP_DMS_ENABLE
    printf("dms_thread: compiled out (APP_DMS_ENABLE=0)\n");
    return TD_FAILURE;
#endif

    if (g_created == TD_TRUE) {
        printf("dms_thread: already running\n");
        return TD_FAILURE;
    }

    if (dms_sink_init() != TD_SUCCESS) {
        return TD_FAILURE;
    }

    /* NPU 运行环境初始化：整个进程只有 DMS 用 NPU，失败则 DMS 不启动。 */
    if (npu_model_global_init() != TD_SUCCESS) {
        printf("dms_thread: npu init failed, DMS disabled\n");
        dms_sink_deinit();
        return TD_FAILURE;
    }

    if (dms_models_init(&g_models) != TD_SUCCESS) {
        printf("dms_thread: model init failed, DMS disabled\n");
        goto cleanup;
    }

    dms_metrics_init(&g_metrics);
    if (g_metrics.ring.buf == NULL) {
        printf("dms_thread: metrics init failed, DMS disabled\n");
        goto cleanup;
    }
    dms_tracker_init(&g_tracker);
    g_frames = 0;
    g_face_present = TD_FALSE;
    atomic_store(&g_stop, TD_FALSE);

    /* 声光报警（LED + 蜂鸣器）：失败只少了提示，不影响 DMS 主功能。 */
    if (dms_alert_init() != TD_SUCCESS) {
        printf("dms_thread: alert output not started, continue\n");
    }

    /* 先让事件录像订阅 H.264，开始积累预录帧。订阅失败时仍运行判定。 */
    if (dms_clip_start() != TD_SUCCESS) {
        printf("dms_thread: clip recorder not started, continue\n");
    }

    thread_ret = pthread_create(&g_thread, NULL, dms_thread_main, NULL);
    if (thread_ret != 0) {
        printf("dms_thread: pthread_create failed: %d\n", thread_ret);
        goto cleanup;
    }
    (void)pthread_setname_np(g_thread, "dms_thread");

    g_created = TD_TRUE;
    printf("dms_thread_start success\n");
    return TD_SUCCESS;

cleanup:
    dms_alert_deinit();
    dms_clip_stop();
    dms_models_deinit(&g_models);
    dms_metrics_deinit(&g_metrics);
    npu_model_global_deinit();
    dms_sink_deinit();
    return TD_FAILURE;
}

/* 等待 DMS 线程退出，再关事件录像、声光输出并释放模型和判定状态。 */
td_s32 dms_thread_stop(void)
{
    int thread_ret;

    if (g_created == TD_FALSE) {
        return TD_SUCCESS;
    }

    atomic_store(&g_stop, TD_TRUE);
    thread_ret = pthread_join(g_thread, NULL);
    if (thread_ret != 0) {
        printf("dms_thread: pthread_join failed: %d\n", thread_ret);
        return TD_FAILURE;
    }
    g_created = TD_FALSE;

    dms_alert_deinit();
    dms_clip_stop();
    dms_models_deinit(&g_models);
    dms_metrics_deinit(&g_metrics);
    npu_model_global_deinit();
    dms_sink_deinit();

    printf("dms_thread_stop success, total frames=%u\n", g_frames);
    return TD_SUCCESS;
}
