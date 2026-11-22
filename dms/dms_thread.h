/*
 * dms_thread：DMS 后台线程的启动/停止。
 *   用法：
 *     start：主线程里加载模型、准备状态，再创建线程
 *     stop ：先通知线程退出 → pthread_join → 释放模型
 *   依赖 VPSS 通道 2（640x640）在运行，所以必须在 video_processing_stop() 之前 stop。
 *
 *   隔离性：DMS 内部任何一步出错都只让 DMS 线程退出或跳过这一帧，
 *   不会影响采图/编码/RTSP/录像主流程。
 */
#ifndef DMS_THREAD_H
#define DMS_THREAD_H

#include "ot_type.h"

td_s32 dms_thread_start(void);
td_s32 dms_thread_stop(void);

#endif /* DMS_THREAD_H */
