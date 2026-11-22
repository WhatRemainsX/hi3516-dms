/*
 * dms_detect.c：dms_detect.h 的实现。
 * 流水线位置：DMS 第二站，负责把 SCRFD 的原始输出解码成人脸框并做跟踪。
 * 输入：已执行完的 SCRFD 输出张量和上一帧跟踪框。
 * 输出：人脸框、外扩框、NMS 结果和跟踪状态；由 dms_models/dms_thread 调用。
 *   解码约定来自 SCRFD 官方参考实现（insightface/detection/scrfd/tools/scrfd.py）：
 *     - 预处理：RGB（官方 blobFromImage swapRB=True），(v-127.5)/128，输入 640x640
 *     - anchor 中心 = (grid_x*stride, grid_y*stride)
 *     - bbox 输出 = (l,t,r,b)*stride，x1=cx-l, y1=cy-t, x2=cx+r, y2=cy+b
 *     - 每个格子 2 个 anchor，同一格子的两个 anchor 在输出里相邻
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "dms_detect.h"
#include "app_config.h"

#define DMS_STRIDE_LEVELS 3
static const td_u32 DMS_STRIDES[3] = {8, 16, 32};

/* 解码中间结果。16800 = 12800+3200+800，是 SCRFD-500M 全部 anchor 的上限。 */
#define DMS_RAW_MAX_CANDIDATES 16800
static dms_box g_raw[DMS_RAW_MAX_CANDIDATES];
static td_u8 g_suppressed[DMS_RAW_MAX_CANDIDATES];

static td_u32 dms_detect_run_nms(td_u32 count, float iou_thresh, dms_box *boxes, td_u32 max_boxes);


/* 一个 score 输出和一个 bbox 输出配成一对。 */
typedef struct {
    td_u32 score_index;
    td_u32 bbox_index;
    td_u32 rows;
    td_u32 stride;
} dms_level;

/* 从一个输出张量里取出"最后一维"和"行数"。 */
static void output_shape(const npu_tensor_info *info, td_u32 *rows, td_u32 *last_dim)
{
    *last_dim = 0;
    *rows = 0;
    if (info->dim_count < 2) {
        return;
    }
    *rows = (td_u32)info->dims[info->dim_count - 2];
    *last_dim = (td_u32)info->dims[info->dim_count - 1];
}

/* 按行数认步长：640/stride 的格子数 × 2。认不出返回 0。 */
static td_u32 stride_from_rows(td_u32 rows)
{
    td_u32 level;
    for (level = 0; level < DMS_STRIDE_LEVELS; level++) {
        td_u32 stride = DMS_STRIDES[level];
        td_u32 grid = APP_DMS_IN_WIDTH / stride;   /* 640/stride */
        if (grid * grid * 2 == rows) {
            return stride;
        }
    }
    return 0;
}

td_u32 dms_detect_decode(const npu_model *detector, float score_thresh, float iou_thresh,
                         dms_box *boxes, td_u32 max_boxes)
{
    dms_level levels[DMS_STRIDE_LEVELS];
    td_u32 level_count = 0;
    td_u32 output_count;
    td_u32 level;
    td_u32 index;
    td_u32 raw_count = 0;
    td_u32 result;

    if (detector == NULL || boxes == NULL || max_boxes == 0) {
        return 0;
    }

    /* [步骤 1/3] 遍历所有输出，按形状把 score 与 bbox 配到 8/16/32 三档。 */
    output_count = npu_model_get_output_count(detector);
    for (index = 0; index < output_count; index++) {
        npu_tensor_info info;
        td_u32 rows;
        td_u32 last_dim;
        td_u32 stride;
        td_u32 level;
        td_s32 ret;

        ret = npu_model_get_output_info(detector, index, &info);
        if (ret != TD_SUCCESS) {
            continue;
        }
        output_shape(&info, &rows, &last_dim);
        stride = stride_from_rows(rows);
        if (stride == 0) {
            continue;   /* 不是 SCRFD 的 anchor 输出 */
        }

        if (last_dim == 1) {
            /* score 输出：找同一 strides 的 bbox 输出配对。 */
            for (level = 0; level < level_count; level++) {
                if (levels[level].stride == stride && levels[level].score_index == 0xFFFFFFFF) {
                    levels[level].score_index = index;
                    levels[level].rows = rows;
                    break;
                }
            }
            if (level == level_count && level_count < DMS_STRIDE_LEVELS) {
                levels[level_count].score_index = index;
                levels[level_count].bbox_index = 0xFFFFFFFF;
                levels[level_count].rows = rows;
                levels[level_count].stride = stride;
                level_count++;
            }
        } else if (last_dim == 4) {
            for (level = 0; level < level_count; level++) {
                if (levels[level].stride == stride && levels[level].bbox_index == 0xFFFFFFFF) {
                    levels[level].bbox_index = index;
                    break;
                }
            }
            if (level == level_count && level_count < DMS_STRIDE_LEVELS) {
                levels[level_count].score_index = 0xFFFFFFFF;
                levels[level_count].bbox_index = index;
                levels[level_count].rows = rows;
                levels[level_count].stride = stride;
                level_count++;
            }
        }
    }

    if (level_count == 0) {
        printf("dms_detect: no SCRFD outputs recognised\n");
        return 0;
    }

    /* [步骤 2/3] 三档分别解码：anchor 中心加上四边偏移，得到原图框。 */
    for (level = 0; level < level_count; level++) {
        npu_tensor_info score_info;
        npu_tensor_info bbox_info;
        const td_u8 *score_base;
        const td_u8 *bbox_base;
        td_s32 ret;
        td_u32 row;
        td_u32 grid_width;
        td_u32 stride = levels[level].stride;

        if (levels[level].score_index == 0xFFFFFFFF || levels[level].bbox_index == 0xFFFFFFFF) {
            continue;
        }
        ret = npu_model_get_output_info(detector, levels[level].score_index, &score_info);
        if (ret != TD_SUCCESS) {
            continue;
        }
        ret = npu_model_get_output_info(detector, levels[level].bbox_index, &bbox_info);
        if (ret != TD_SUCCESS) {
            continue;
        }
        score_base = (const td_u8 *)npu_model_get_output_buffer(detector, levels[level].score_index);
        bbox_base = (const td_u8 *)npu_model_get_output_buffer(detector, levels[level].bbox_index);
        if (score_base == NULL || bbox_base == NULL) {
            continue;
        }

        grid_width = APP_DMS_IN_WIDTH / stride;
        for (row = 0; row < levels[level].rows; row++) {
            const float *score_ptr;
            const float *bbox_ptr;
            td_u32 cell;
            td_u32 grid_x;
            td_u32 grid_y;
            float cx;
            float cy;
            float l;
            float t;
            float r;
            float b;

            score_ptr = (const float *)(score_base + (size_t)row * score_info.stride);
            if (score_ptr[0] < score_thresh) {
                continue;
            }
            if (raw_count >= DMS_RAW_MAX_CANDIDATES) {
                break;
            }
            cell = row / 2;                     /* 同一格子的 2 个 anchor 相邻 */
            grid_x = cell % grid_width;
            grid_y = cell / grid_width;
            cx = (float)(grid_x * stride);
            cy = (float)(grid_y * stride);

            bbox_ptr = (const float *)(bbox_base + (size_t)row * bbox_info.stride);
            l = bbox_ptr[0] * (float)stride;
            t = bbox_ptr[1] * (float)stride;
            r = bbox_ptr[2] * (float)stride;
            b = bbox_ptr[3] * (float)stride;

            g_raw[raw_count].x1 = cx - l;
            g_raw[raw_count].y1 = cy - t;
            g_raw[raw_count].x2 = cx + r;
            g_raw[raw_count].y2 = cy + b;
            g_raw[raw_count].score = score_ptr[0];
            raw_count++;
        }
    }

    /* [步骤 3/3] 初始框远多于驾驶员数量，用 NMS 按分数保留不重叠框。 */
    result = dms_detect_run_nms(raw_count, iou_thresh, boxes, max_boxes);
    return result;
}

/* qsort 比较：分数高的在前。 */
static int dms_compare_by_score_desc(const void *a, const void *b)
{
    const dms_box *ba = (const dms_box *)a;
    const dms_box *bb = (const dms_box *)b;

    if (ba->score > bb->score) {
        return -1;
    }
    if (ba->score < bb->score) {
        return 1;
    }
    return 0;
}

td_u32 dms_detect_run_nms(td_u32 count, float iou_thresh, dms_box *boxes, td_u32 max_boxes)
{
    td_u32 output_count = 0;
    td_u32 i;
    td_u32 j;

    if (count == 0) {
        return 0;
    }
    memset(g_suppressed, 0, count * sizeof(g_suppressed[0]));
    qsort(g_raw, count, sizeof(g_raw[0]), dms_compare_by_score_desc);

    for (i = 0; i < count; i++) {
        if (g_suppressed[i] != 0) {
            continue;
        }
        if (output_count >= max_boxes) {
            break;
        }
        boxes[output_count] = g_raw[i];
        output_count++;
        for (j = i + 1; j < count; j++) {
            if (g_suppressed[j] != 0) {
                continue;
            }
            if (dms_detect_iou(&g_raw[i], &g_raw[j]) > iou_thresh) {
                g_suppressed[j] = 1;
            }
        }
    }
    return output_count;
}

/* 输入：两个矩形框。输出：交集面积除以并集面积，范围 0~1。 */
float dms_detect_iou(const dms_box *a, const dms_box *b)
{
    float ax1;
    float ay1;
    float ax2;
    float ay2;
    float inter_w;
    float inter_h;
    float inter_area;
    float area_a;
    float area_b;
    float union_area;

    ax1 = a->x1 > b->x1 ? a->x1 : b->x1;
    ay1 = a->y1 > b->y1 ? a->y1 : b->y1;
    ax2 = a->x2 < b->x2 ? a->x2 : b->x2;
    ay2 = a->y2 < b->y2 ? a->y2 : b->y2;
    inter_w = ax2 - ax1;
    inter_h = ay2 - ay1;
    if (inter_w < 0.0f) {
        inter_w = 0.0f;
    }
    if (inter_h < 0.0f) {
        inter_h = 0.0f;
    }
    inter_area = inter_w * inter_h;
    area_a = (a->x2 - a->x1) * (a->y2 - a->y1);
    area_b = (b->x2 - b->x1) * (b->y2 - b->y1);
    union_area = area_a + area_b - inter_area;
    if (union_area <= 0.0f) {
        return 0.0f;
    }
    return inter_area / union_area;
}

/*
 * 输入：原框、外扩比例和图像宽高。
 * 输出：以原框中心向四周等比扩大的框，并裁在图像边界内。
 * 注意：关键点模型需要嘴边/眼角留白，不扩框时边缘点容易预测不准。
 */
void dms_detect_expand(const dms_box *in, float ratio, float img_w, float img_h, dms_box *out)
{
    float cx = (in->x1 + in->x2) * 0.5f;
    float cy = (in->y1 + in->y2) * 0.5f;
    float hw = (in->x2 - in->x1) * 0.5f * (1.0f + ratio);
    float hh = (in->y2 - in->y1) * 0.5f * (1.0f + ratio);

    out->x1 = cx - hw;
    out->y1 = cy - hh;
    out->x2 = cx + hw;
    out->y2 = cy + hh;
    out->score = in->score;

    if (out->x1 < 0.0f) {
        out->x1 = 0.0f;
    }
    if (out->y1 < 0.0f) {
        out->y1 = 0.0f;
    }
    if (out->x2 > img_w) {
        out->x2 = img_w;
    }
    if (out->y2 > img_h) {
        out->y2 = img_h;
    }
}

/* 输入：跟踪器对象。输出：清零跟踪状态，第一次检测会把它当作新目标。 */
void dms_tracker_init(dms_tracker *tr)
{
    memset(tr, 0, sizeof(*tr));
}

/*
 * 输入：本帧检测框、IoU 匹配门限和旧框平滑系数。
 * 输出：更新跟踪框；匹配不上时选面积最大的框，默认最大的脸才是驾驶员。
 * 注意：smooth 越大旧框权重越高，画面更稳但跟随会更慢。
 */
void dms_tracker_on_detect(dms_tracker *tr, const dms_box *boxes, td_u32 count,
                           float iou_thresh, float smooth)
{
    td_u32 index;
    td_u32 best_index = 0;
    td_bool matched = TD_FALSE;

    if (count == 0) {
        tr->missed++;
        return;
    }

    if (tr->valid) {
        /* 已跟踪：找和当前框 IoU 最大的检测。 */
        float best_iou = 0.0f;
        for (index = 0; index < count; index++) {
            float iou = dms_detect_iou(&tr->box, &boxes[index]);
            if (iou > best_iou) {
                best_iou = iou;
                best_index = index;
            }
        }
        if (best_iou >= iou_thresh) {
            matched = TD_TRUE;
        }
    }

    if (!matched) {
        /* 没匹配上（首次或跟丢）：取面积最大的框作为驾驶员。 */
        float best_area = -1.0f;
        for (index = 0; index < count; index++) {
            float area = (boxes[index].x2 - boxes[index].x1) *
                         (boxes[index].y2 - boxes[index].y1);
            if (area > best_area) {
                best_area = area;
                best_index = index;
            }
        }
    }

    tr->last_det = boxes[best_index];
    /* 只有和旧框匹配上（同一张脸）才做平滑；没匹配上说明是新目标或跟丢后重新
     * 出现，旧框已无意义，直接用新框（否则会把新框往旧位置拉 60%，关键点裁偏）。 */
    if (tr->valid && matched) {
        /* 指数平滑：新框占 (1-smooth)，旧框占 smooth。 */
        tr->box.x1 = smooth * tr->box.x1 + (1.0f - smooth) * boxes[best_index].x1;
        tr->box.y1 = smooth * tr->box.y1 + (1.0f - smooth) * boxes[best_index].y1;
        tr->box.x2 = smooth * tr->box.x2 + (1.0f - smooth) * boxes[best_index].x2;
        tr->box.y2 = smooth * tr->box.y2 + (1.0f - smooth) * boxes[best_index].y2;
    } else {
        tr->box = boxes[best_index];
    }
    tr->box.score = boxes[best_index].score;
    tr->valid = TD_TRUE;
    tr->missed = 0;
}

/* 输入：跟踪器对象。输出：漏检计数加一；调用方超过上限后会丢弃跟踪。 */
void dms_tracker_on_skip(dms_tracker *tr)
{
    /* 中间帧没有新的检测结果：框保持不变，只记一次漏检。
     * 连续漏检太多时由调用方决定丢弃（dms_thread 里处理）。 */
    tr->missed++;
}
