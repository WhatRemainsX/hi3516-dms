/*
 * dms_models：DMS 用到的 3 个 NPU 模型的加载、预处理和推理（头姿走 PnP，不再用 NPU）。
 * 流水线位置：DMS 第二站到第三站之间的接口，提供检测、关键点、头姿和眼睛结果。
 * 输入：640x640 NV21 帧、人脸框和 98 点关键点。
 * 输出：检测框、关键点、yaw/pitch/roll 和左右眼状态；由 dms_thread 调用。
 *   det  : SCRFD-500M  输入 1x3x640x640  RGB，(v-127.5)/128
 *   lm   : pfld_98.onnx（WFLW 98 点）  输入 1x3x112x112  BGR，像素/255（0~1），框长边×1.1 正方形裁剪
 *   eye  : open-closed-eye-0001         输入 1x3x32x32   BGR，(像素-127)/255
 *   pose : 已弃用（原 head-pose-estimation-adas-0001），头姿改用关键点 PnP 解算。
 *
 * 预处理约定来自 OpenVINO gaze estimation demo（见 DMS_REPORT.md 的引用）：
 *   图像统一 resize 到模型输入尺寸，不做均值/方差归一化，颜色顺序见上表。
 *   PFLD 的预处理（RGB、像素/255）与其训练一致，见 pfld-stage-b-TASK.md。
 *
 * 线程：内部没有锁，和 npu_model 一样只能被一个线程使用（DMS 线程）。
 */
#ifndef DMS_MODELS_H
#define DMS_MODELS_H

#include "dms_types.h"
#include "npu_model.h"
#include "ot_common_video.h"   /* ot_video_frame */

typedef struct {
    npu_model *det;
    npu_model *lm;
    npu_model *eye;
    npu_model *pose;    /* 已弃用，恒为 NULL（头姿走 PnP）；保留字段仅为兼容 */
} dms_models;

/* PnP 失败分支（调用方可传 dms_pose_debug 取回，排查用；正常运行传 NULL）。 */
typedef enum {
    DMS_POSE_FAIL_NONE = 0,
    DMS_POSE_FAIL_INPUT,
    DMS_POSE_FAIL_LANDMARK,
    DMS_POSE_FAIL_NONFINITE,
    DMS_POSE_FAIL_SOLVE,
    DMS_POSE_FAIL_DIVERGE,
    DMS_POSE_FAIL_COST,
    DMS_POSE_FAIL_RANGE
} dms_pose_fail_reason;

typedef struct {
    float cost;
    td_s32 iterations;
    dms_pose_fail_reason reason;
} dms_pose_debug;

/* 眼分类器原始输出和实际裁剪框参数。 */
typedef struct {
    float out0;
    float out1;
    float cx;
    float cy;
    float side;
    td_bool valid;
} dms_eye_debug;

/* 加载全部模型并核对输入尺寸。任一失败都会把已加载的卸掉，返回 TD_FAILURE。
 * 输入：调用方提供的 dms_models 对象。
 * 输出：成功时模型可直接推理；失败时返回 TD_FAILURE。 */
td_s32 dms_models_init(dms_models *m);

/* 输入：已初始化的模型对象。输出：按反序卸载模型；重复调用是安全的。 */
void dms_models_deinit(dms_models *m);

/* 清除 PnP 热启动历史，下一帧从瞳孔连线冷启动。 */
void dms_models_reset_pose(void);

/* 检测：把 640x640 NV21 帧写成 SCRFD 输入，执行，解码 + NMS。
 * 输入：帧和最多返回框数；输出：检测框数组及数量；
 * 注意：SCRFD 执行失败返回 TD_FAILURE。 */
td_s32 dms_models_detect(dms_models *m, const ot_video_frame *frame,
                         dms_box *boxes, td_u32 max_boxes, td_u32 *count);

/* 关键点：从帧里裁出 box（640x640 坐标），resize 到 112x112，输出 98 点（640x640 坐标）。
 * 输入：帧和人脸框；输出：98 点；注意：裁剪/缩放失败时返回 TD_FAILURE。 */
td_s32 dms_models_run_landmarks(dms_models *m, const ot_video_frame *frame,
                                const dms_box *box, dms_landmarks *lm);

/* 头姿：用 98 点关键点（640x640 坐标）做 PnP 解算，输出 yaw/pitch/roll（度）。
 * 输入：98 点和可选诊断；输出：三个角度；注意：失败时角度清零并返回 TD_FAILURE。
 * 符号：yaw>0 为鼻尖朝画面右，pitch>0 为抬头（<0 低头），roll>0 为画面顺时针。 */
td_s32 dms_models_run_pose(const dms_landmarks *lm,
                           float *yaw, float *pitch, float *roll,
                           dms_pose_debug *debug);

/* 眼睛：用 98 点眼组（左眼 68-75、右眼 60-67）的两眼角点裁眼框，按 roll 旋转后
 * resize 到 32x32 分类。side0 = 左眼（对应 *left），side1 = 右眼（对应 *right）。
 * 输入：帧/关键点/roll；输出：左右眼状态；注意：歪头时必须转正，否则分类器看不清。 */
td_s32 dms_models_run_eye(dms_models *m, const ot_video_frame *frame,
                          const dms_landmarks *lm, float roll,
                          dms_eye_state *left, dms_eye_state *right,
                          dms_eye_debug debug[2]);

#endif /* DMS_MODELS_H */
