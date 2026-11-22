/*
 * 程序入口，负责按依赖顺序启动和关闭相机模块。
 *
 * SC4336P → MIPI → VI 采集（含 ISP 图像处理）→ VPSS 缩放和分流
 *   VPSS 通道 0 → VENC 编码 H.264 → 取流线程 → media_source
 *       ├─ RTSP 客户端的 RTP 发送器
 *       ├─ 常规录像 rec.mp4
 *       └─ 事件录像 dms_event_*.mp4
 *   VPSS 通道 2 → DMS 线程 → 驾驶员状态、报警事件和录像请求
 *
 * 两路在事件录像处汇合。media_source 提供编码画面，DMS 事件决定何时
 * 保存片段。实际取流、推理和写文件由各模块的线程执行。
 */

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "app_config.h"
#include "dms_sink.h"
#include "dms_thread.h"
#include "isp_control.h"
#include "media_system.h"
#include "mp4_recorder.h"
#include "rtsp_server.h"
#include "video_capture.h"
#include "video_encoder.h"
#include "video_osd.h"
#include "video_processing.h"

/* 信号处理函数写这个标志，主线程读取后执行清理。 */
static volatile sig_atomic_t g_quit = 0;

static void on_signal(int sig)
{
    (void)sig;
    g_quit = 1;
}

/* Ctrl+C 发送 SIGINT，kill 默认发送 SIGTERM。两者共用退出流程。 */
static int install_signal_handlers(void)
{
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESETHAND;   /* 同一信号再次到来时使用默认处理。 */

    if (sigaction(SIGINT, &sa, NULL) != 0 ||
        sigaction(SIGTERM, &sa, NULL) != 0) {
        perror("sigaction");
        return -1;
    }
    return 0;
}

int main(void)
{
    /* 清理操作使用单独的返回值，避免覆盖已有的启动或运行结果。 */
    td_s32 ret;
    td_s32 cleanup_ret;
    rtsp_server *rtsp = NULL;
    mp4_recorder *recorder = NULL;

    if (install_signal_handlers() != 0) {
        return 1;
    }

    /* 1. 初始化媒体系统。后续采集和处理模块都使用 SYS 和公共视频缓冲池 VB。 */
    ret = media_system_init();
    if (ret != TD_SUCCESS) {
        return 1;
    }

    /*
     * 2. 建立采集链路。MIPI 负责把传感器的数据送进 VI。
     *    VI 设备接收数据，再经过 Pipe 和通道输出图像。
     *    出错时跳到下方能处理已创建资源的清理入口。
     */
    ret = video_capture_set_mipi_lane_mode();
    if (ret != TD_SUCCESS) {
        goto exit_system;
    }

    ret = video_capture_start_sensor();
    if (ret != TD_SUCCESS) {
        goto exit_system;
    }

    ret = video_capture_start_mipi();
    if (ret != TD_SUCCESS) {
        goto stop_sensor;
    }

    ret = video_capture_start_vi_dev();
    if (ret != TD_SUCCESS) {
        goto stop_mipi;
    }

    ret = video_capture_bind_pipe();
    if (ret != TD_SUCCESS) {
        goto stop_vi_dev;
    }

    ret = video_capture_set_vi_group();
    if (ret != TD_SUCCESS) {
        goto unbind_pipe;
    }

    ret = video_capture_start_pipe();
    if (ret != TD_SUCCESS) {
        goto unbind_pipe;
    }

    ret = video_capture_start_chn();
    if (ret != TD_SUCCESS) {
        goto stop_pipe;
    }

    /*
     * 3. VI 就绪后启动 ISP。先注册传感器和曝光、白平衡算法，
     *    再配置 ISP，最后启动负责持续图像处理的线程。
     */
    ret = isp_control_register();
    if (ret != TD_SUCCESS) {
        goto stop_chn;
    }

    ret = isp_control_init();
    if (ret != TD_SUCCESS) {
        goto deinit_isp;
    }

    ret = isp_control_start();
    if (ret != TD_SUCCESS) {
        goto deinit_isp;
    }

    printf("ISP started successfully\n");
    printf("Media system initialized, VI Pipe 0 Channel 0 started, and ISP running\n");
    printf("starting VPSS Group 0 Channel 0...\n");

    /* 4. 先建好 VPSS 输出通道，再把 VI 的图像输出绑定到 VPSS 输入。 */
    ret = video_processing_start();
    if (ret != TD_SUCCESS) {
        goto deinit_isp;
    }

    ret = video_processing_bind_vi();
    if (ret != TD_SUCCESS) {
        goto stop_vpss;
    }

    /*
     * 5. VENC 先准备好接收，再绑定 VPSS 通道 0。取流线程负责从 VENC
     *    拿到 H.264，写裸码流文件 stream.h264，并通过 media_source 分发。
     */
    ret = video_encoder_create();
    if (ret != TD_SUCCESS) {
        goto unbind_vpss;
    }

    ret = video_encoder_start_receive();
    if (ret != TD_SUCCESS) {
        goto destroy_venc;
    }

    ret = video_encoder_bind_vpss();
    if (ret != TD_SUCCESS) {
        goto stop_venc_receive;
    }

    ret = video_encoder_start_stream(APP_OUTPUT_DIR "/stream.h264");
    if (ret != TD_SUCCESS) {
        goto stop_venc_stream;
    }

    /* 6. 启动视频输出。RTSP 客户端发出 PLAY 后，RTP 发送器才开始订阅码流。 */
    rtsp = rtsp_server_start(APP_RTSP_PORT);
    if (rtsp == NULL) {
        printf("Failed to start RTSP server\n");
    }

    /*
     * 常规录像也是码流订阅者，从可解码的关键帧开始写。时长传 0 表示
     * 一直录到程序关闭。常规录像初始化失败会进入清理流程。
     */
    recorder = mp4_recorder_start(APP_OUTPUT_DIR "/rec.mp4",
                                  APP_VPSS_OUT_WIDTH, APP_VPSS_OUT_HEIGHT,
                                  0);
    if (recorder == NULL) {
        printf("Failed to start MP4 recorder\n");
        goto stop_video_outputs;
    }

    /*
     * 7. 启动 DMS。VPSS 通道 2 提供推理画面，事件录像另行订阅 H.264。
     *    到这一步如果 DMS 启动失败，已经运行的输出会继续工作。
     */
    ret = dms_thread_start();
    if (ret != TD_SUCCESS) {
        printf("Failed to start DMS thread (camera keeps running)\n");
    } else {
        /* dms_thread_start() 会初始化回调，所以启动成功后再接画面叠加。 */
        video_osd_init();
        dms_sink_set_overlay_hook(video_osd_set_text, NULL);
    }

    /* 后台线程处理画面。主线程每秒检查退出标志，收到信号后开始清理。 */
    printf("Streaming... press Ctrl+C to quit\n");
    fflush(stdout);

    while (g_quit == 0) {
        sleep(1);
    }
    printf("\nSignal received, shutting down...\n");

    /* DMS 依赖 VPSS 和 OSD，先停止 DMS 线程，再释放这些资源。 */
    dms_thread_stop();
    video_osd_deinit();
    goto stop_video_outputs;

    /*
     * 退出或启动失败时，进入对应的标签并继续向下执行。
     * 先停码流订阅者，再停止取流、解绑硬件并逐级释放资源。
     */
stop_video_outputs:
    /* 这两个停止函数都接受 NULL，启动到一半时也可使用同一个清理入口。 */
    mp4_recorder_stop(recorder);
    rtsp_server_stop(rtsp);

stop_venc_stream:
    /* 等取流线程结束之后，才可以安全地解绑和关闭 VENC。 */
    cleanup_ret = video_encoder_stop_stream();
    if (cleanup_ret != TD_SUCCESS) {
        printf("VENC stream cleanup failed; remaining teardown skipped\n");
        return 1;
    }

    /* 取流或裸码流写入失败时，记录运行错误并继续清理。 */
    if (video_encoder_get_stream_result() != TD_SUCCESS) {
        printf("VENC recording failed\n");
        ret = TD_FAILURE;
    }

    cleanup_ret = video_encoder_unbind_vpss();
    if (cleanup_ret != TD_SUCCESS) {
        printf("VENC unbind failed; remaining teardown skipped\n");
        return 1;
    }

stop_venc_receive:
    cleanup_ret = video_encoder_stop_receive();
    if (cleanup_ret != TD_SUCCESS) {
        printf("VENC stop failed; remaining teardown skipped\n");
        return 1;
    }

destroy_venc:
    cleanup_ret = video_encoder_destroy();
    if (cleanup_ret != TD_SUCCESS) {
        printf("VENC destroy failed; remaining teardown skipped\n");
        return 1;
    }

unbind_vpss:
    video_processing_unbind_vi();

stop_vpss:
    video_processing_stop();

deinit_isp:
    /* 等 ISP 线程退出，再注销它使用的传感器和算法。 */
    if (isp_control_deinit() != TD_SUCCESS) {
        printf("ISP cleanup failed; remaining teardown skipped\n");
        return 1;
    }
    isp_control_unregister();

stop_chn:
    video_capture_stop_chn();
stop_pipe:
    video_capture_stop_pipe();
unbind_pipe:
    video_capture_unbind_pipe();
stop_vi_dev:
    video_capture_stop_vi_dev();
stop_mipi:
    video_capture_stop_mipi();
stop_sensor:
    video_capture_stop_sensor();

exit_system:
    /* 媒体系统最先初始化，最后释放。返回 0 表示成功，1 表示失败。 */
    media_system_deinit();
    return (ret == TD_SUCCESS) ? 0 : 1;
}
