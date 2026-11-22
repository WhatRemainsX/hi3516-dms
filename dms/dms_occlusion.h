/*
 * dms_occlusion：摄像头遮挡自检（占位实现，TODO：实车标定后接入分级报警）。
 */
#ifndef DMS_OCCLUSION_H
#define DMS_OCCLUSION_H

#include "ot_common_video.h"   /* ot_video_frame */

/* 按亮度方差判断画面是否被遮挡；疑似遮挡时打印日志。 */
void dms_occlusion_check(const ot_video_frame *frame);

#endif /* DMS_OCCLUSION_H */
