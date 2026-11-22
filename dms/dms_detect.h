/*
 * dms_detect：SCRFD 检测输出的解码 + 单目标跟踪。
 *
 * SCRFD-500M 的 ONNX 有 6 个输出：
 *   score_8  [1,12800,1]  / bbox_8  [1,12800,4]
 *   score_16 [1,3200,1]   / bbox_16 [1,3200,4]
 *   score_32 [1,800,1]    / bbox_32 [1,800,4]
 * 这里不写死输出下标，而是按"最后一个维度是 1 还是 4"来认 score/bbox，
 * 再按行数对应步长 8/16/32，这样 .om 转换后即使输出顺序变了也能工作。
 *
 * 线程：内部用静态缓冲区，和 npu_model 一样不是线程安全的，只允许一个线程使用。
 */
#ifndef DMS_DETECT_H
#define DMS_DETECT_H

#include "dms_types.h"
#include "npu_model.h"

/* 解码一帧检测输出，NMS 后写进 boxes，返回框的数量。 */
td_u32 dms_detect_decode(const npu_model *detector, float score_thresh, float iou_thresh,
                         dms_box *boxes, td_u32 max_boxes);

float dms_detect_iou(const dms_box *a, const dms_box *b);


/* 把框按比例外扩（绕中心），并夹到画面范围内。 */
void dms_detect_expand(const dms_box *in, float ratio, float img_w, float img_h, dms_box *out);

/* 单目标跟踪器：只跟"驾驶员"（最大的人脸）。
 *   on_detect：检测到结果的帧调用，做匹配 + 平滑
 *   on_skip  ：中间帧调用，保持框不变，只累加 missed */
typedef struct {
    td_bool valid;
    dms_box box;        /* 平滑后的框 */
    dms_box last_det;   /* 最近一次原始检测结果 */
    td_u32 missed;
} dms_tracker;

void dms_tracker_init(dms_tracker *tr);
void dms_tracker_on_detect(dms_tracker *tr, const dms_box *boxes, td_u32 count,
                           float iou_thresh, float smooth);
void dms_tracker_on_skip(dms_tracker *tr);

#endif /* DMS_DETECT_H */
