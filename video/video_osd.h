/*
 * video_osd：RTSP 画面的文字叠加（OSD）接口。
 *
 *   main.c 把 dms_sink_set_overlay_hook(video_osd_set_text, NULL) 接过来，
 *   DMS 每秒产生的状态文字就流到这里。
 *
 * 【现在的状态】APP_DMS_OSD_RGN=1（见 app_config.h）：用 SDK REGION 模块在
 *   VENC 通道 0 上挂一个 OVERLAY 区域画字，1Hz 刷新；set_text 只更新缓存，
 *   实际渲染由内部 1Hz 线程完成。RGN 初始化/刷新失败只记日志，不影响主流程。
 * 【依据】真实头文件 ss_mpi_region.h / ot_common_region.h / ot_common_video.h
 *   与 SDK 样例 kol/region/region.c、common/sample_comm_region.c，
 *   对照明细见 video_osd.c 文件头注释。
 */
#ifndef VIDEO_OSD_H
#define VIDEO_OSD_H

#include "ot_type.h"

/* 初始化 OSD（清缓存；RGN=1 时还会创建区域/位图资源）。 */
td_s32 video_osd_init(void);

/* 设置要显示的文字（DMS 每秒调一次）；text 为 NULL 时忽略；arg 由钩子传入，这里不用。 */
void video_osd_set_text(const char *text, void *arg);

/* 释放 OSD 资源（RGN=1 时解除绑定并销毁区域）。 */
void video_osd_deinit(void);

#endif /* VIDEO_OSD_H */
