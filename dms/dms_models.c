/*
 * dms_models.c：dms_models.h 的实现。
 * 流水线位置：DMS 第二站到第三站，负责模型输入预处理、NPU 推理和关键点头姿解算。
 * 输入：640x640 NV21 帧、人脸框；输出：检测框、98 点、左右眼状态和头姿角。
 * 调用关系：dms_thread 每帧调用；NPU 模型相关状态只能由 DMS 单线程访问。
 *   裁剪 + 双线性缩放 + NV21→BGR/RGB 都在这里：
 *     - 图像统一缩放到模型输入尺寸，不做均值/方差归一化
 *     - 关键点（PFLD 98 点）：BGR，像素/255（0~1），裁剪 = 检测框长边 ×1.1 的正方形
 *     - 眼睛：BGR，(像素-127)/255（约 -0.5~0.5，与官方 OMZ 转换参数一致）
 *     - SCRFD：RGB，(v-127.5)/128
 *   坐标映射用 OpenCV resize 的约定：输出像素中心 (ox+0.5) 对应输入
 *   x1 + (ox+0.5)*crop_w/out_w，双线性采样把像素中心放在 i+0.5。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "app_config.h"
#include "dms_models.h"
#include "dms_detect.h"

#include "ss_mpi_sys.h"
#include "ss_mpi_sys_mem.h"     /* ss_mpi_sys_mmap_cached / flush_cache / munmap */

#define DMS_DET_SIZE   APP_DMS_IN_WIDTH      /* 640 */
#define DMS_LM_SIZE    112                     /* PFLD 98 点模型输入 112x112 */
#define DMS_EYE_SIZE   32
#define DMS_EYE_SCALE  1.8f                 /* 眼框边长 = 眼角间距 × 1.8（demo 默认值） */
/* PFLD 裁剪：以检测框中心取正方形，边长 = max(w,h)×1.1（PFLD 官方推理脚本的约定）。
 * 2026-11-20 用板端事件录像回放实测（镜像一致性误差，越小越好）：
 *   原做法（框先外扩 15% 再 ×1.3，约 1.5 倍）0.13；×1.1 约 0.04；BGR 比 RGB 再好一点。
 * 原做法人脸在 112x112 里太小，轮廓/嘴/下巴点明显跑偏，导致 PnP 头姿和 MAR 都错。 */
#define DMS_LM_CROP_MARGIN 1.1f
/* 眼睛分类器输入归一化：官方 OMZ 转 IR 时用 mean=127、scale=255，即 (v-127)/255。
 * 原来用 v/255（0~1），输入整体偏移 +0.5，模型输出恒为同一个值（恒判睁眼）。 */
#define DMS_EYE_MEAN   127.0f
#define DMS_EYE_SCALE_DIV 255.0f

/* 小的中间缓冲：112x112 和 32x32 的 planar float。单线程使用，静态即可。 */
static float g_face_buf[3 * DMS_LM_SIZE * DMS_LM_SIZE];
static float g_eye_buf[2 * 3 * DMS_EYE_SIZE * DMS_EYE_SIZE];

/* PnP 上一帧成功解算结果；仅由 DMS 单线程访问。 */
#define PNP_POINT_COUNT 6
#define PNP_PARAM_COUNT 6
static float g_last_pnp[PNP_PARAM_COUNT];
static td_bool g_has_last_pnp = TD_FALSE;

/* 输入：无。输出：清掉 PnP 热启动缓存，下一帧从瞳孔连线冷启动。 */
void dms_models_reset_pose(void)
{
    memset(g_last_pnp, 0, sizeof(g_last_pnp));
    g_has_last_pnp = TD_FALSE;
}

/* 裁剪参数：中心、半宽半高、旋转角（度）。 */
typedef struct {
    float cx;
    float cy;
    float half_w;
    float half_h;
    float rotate_deg;
} dms_crop_params;

/* 检查帧是不是我们需要的 640x640 NV21。 */
static td_s32 dms_check_frame(const ot_video_frame *frame)
{
    if (frame == NULL) {
        return TD_FAILURE;
    }
    if (frame->pixel_format != OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420 ||
        frame->compress_mode != OT_COMPRESS_MODE_NONE ||
        frame->video_format != OT_VIDEO_FORMAT_LINEAR ||
        frame->dynamic_range != OT_DYNAMIC_RANGE_SDR8) {
        printf("dms_models: unsupported frame format\n");
        return TD_FAILURE;
    }
    if (frame->width != APP_DMS_IN_WIDTH || frame->height != APP_DMS_IN_HEIGHT ||
        frame->stride[0] < frame->width || frame->stride[1] < frame->width) {
        printf("dms_models: bad frame size %ux%u\n", frame->width, frame->height);
        return TD_FAILURE;
    }
    return TD_SUCCESS;
}

/*
 * 双线性采样一个平面。像素中心在 i+0.5（和 OpenCV resize 对齐）。
 * pixel_stride 是每个像素占的字节数：Y 平面是 1，NV21 的 VU 交错平面是 2。
 * 需要四个邻点加权，因为直接取最近点会在缩放后产生锯齿；x-0.5 是为了把
 * 输出像素中心对齐到源像素中心，否则整张图会有半像素偏移，关键点会系统性歪。
 */
static float dms_bilinear_plane(const td_u8 *base, td_u32 stride, int pixel_stride,
                                int w, int h, float x, float y)
{
    float xc = x - 0.5f;
    float yc = y - 0.5f;
    int x0 = (int)floorf(xc);
    int y0 = (int)floorf(yc);
    int x1 = x0 + 1;
    int y1 = y0 + 1;
    float fx;
    float fy;
    float v00, v01, v10, v11;
    size_t step = (size_t)pixel_stride;

    if (x0 < 0) { x0 = 0; }
    if (y0 < 0) { y0 = 0; }
    if (x1 < 0) { x1 = 0; }
    if (y1 < 0) { y1 = 0; }
    if (x0 > w - 1) { x0 = w - 1; }
    if (y0 > h - 1) { y0 = h - 1; }
    if (x1 > w - 1) { x1 = w - 1; }
    if (y1 > h - 1) { y1 = h - 1; }

    fx = xc - (float)x0;
    fy = yc - (float)y0;
    if (fx < 0.0f) { fx = 0.0f; }
    if (fx > 1.0f) { fx = 1.0f; }
    if (fy < 0.0f) { fy = 0.0f; }
    if (fy > 1.0f) { fy = 1.0f; }

    v00 = (float)base[(size_t)y0 * stride + (size_t)x0 * step];
    v01 = (float)base[(size_t)y0 * stride + (size_t)x1 * step];
    v10 = (float)base[(size_t)y1 * stride + (size_t)x0 * step];
    v11 = (float)base[(size_t)y1 * stride + (size_t)x1 * step];

    return (v00 * (1.0f - fx) + v01 * fx) * (1.0f - fy) +
           (v10 * (1.0f - fx) + v11 * fx) * fy;
}

/* 在 (x,y) 处采样 NV21，转成 BGR（0~255）。 */
static void dms_sample_nv21(const td_u8 *y_addr, const td_u8 *vu_addr,
                            td_u32 y_stride, td_u32 uv_stride,
                            int w, int h, float x, float y,
                            float *b, float *g, float *r)
{
    float yv;
    float u;
    float v;
    int chroma_w = w / 2;
    int chroma_h = h / 2;
    float cx = x * 0.5f;
    float cy = y * 0.5f;

    yv = dms_bilinear_plane(y_addr, y_stride, 1, w, h, x, y);
    /* NV21 的色度平面是 V、U 交错：偶数字节是 V，奇数字节是 U。 */
    v = dms_bilinear_plane(vu_addr, uv_stride, 2, chroma_w, chroma_h, cx, cy);
    u = dms_bilinear_plane(vu_addr + 1, uv_stride, 2, chroma_w, chroma_h, cx, cy);

    /* BT.601 全范围 YUV → RGB（整数公式）。 */
    *r = yv + 1.402f * (v - 128.0f);
    *g = yv - 0.344136f * (u - 128.0f) - 0.714136f * (v - 128.0f);
    *b = yv + 1.772f * (u - 128.0f);

    if (*r < 0.0f) { *r = 0.0f; }
    if (*r > 255.0f) { *r = 255.0f; }
    if (*g < 0.0f) { *g = 0.0f; }
    if (*g > 255.0f) { *g = 255.0f; }
    if (*b < 0.0f) { *b = 0.0f; }
    if (*b > 255.0f) { *b = 255.0f; }
}

/* 映射 + 刷新帧，失败返回 TD_FAILURE。 */
static td_s32 dms_frame_map(const ot_video_frame *frame, td_u8 **y_addr, td_u8 **vu_addr,
                            td_u32 *y_size, td_u32 *vu_size)
{
    *y_size = frame->stride[0] * frame->height;
    *vu_size = frame->stride[1] * (frame->height / 2);

    *y_addr = (td_u8 *)ss_mpi_sys_mmap_cached(frame->phys_addr[0], *y_size);
    if (*y_addr == NULL) {
        printf("dms_models: map Y plane failed\n");
        return TD_FAILURE;
    }
    *vu_addr = (td_u8 *)ss_mpi_sys_mmap_cached(frame->phys_addr[1], *vu_size);
    if (*vu_addr == NULL) {
        printf("dms_models: map VU plane failed\n");
        ss_mpi_sys_munmap(*y_addr, *y_size);
        *y_addr = NULL;
        return TD_FAILURE;
    }

    if (ss_mpi_sys_flush_cache(frame->phys_addr[0], *y_addr, *y_size) != TD_SUCCESS) {
        printf("dms_models: flush Y cache failed\n");
        ss_mpi_sys_munmap(*vu_addr, *vu_size);
        ss_mpi_sys_munmap(*y_addr, *y_size);
        *y_addr = NULL;
        *vu_addr = NULL;
        return TD_FAILURE;
    }
    if (ss_mpi_sys_flush_cache(frame->phys_addr[1], *vu_addr, *vu_size) != TD_SUCCESS) {
        printf("dms_models: flush VU cache failed\n");
        ss_mpi_sys_munmap(*vu_addr, *vu_size);
        ss_mpi_sys_munmap(*y_addr, *y_size);
        *y_addr = NULL;
        *vu_addr = NULL;
        return TD_FAILURE;
    }
    return TD_SUCCESS;
}

static void dms_frame_unmap(td_u8 *y_addr, td_u8 *vu_addr, td_u32 y_size, td_u32 vu_size)
{
    if (vu_addr != NULL) {
        (void)ss_mpi_sys_munmap(vu_addr, vu_size);
    }
    if (y_addr != NULL) {
        (void)ss_mpi_sys_munmap(y_addr, y_size);
    }
}

/*
 * 把帧按 crop 参数裁出、缩放、转颜色，写进三个平面。
 *   plane0/1/2：三个通道平面的起点，行跨度 stride_bytes。
 *   rgb_order：TD_TRUE 表示 plane0=R、plane1=G、plane2=B；否则 BGR。
 *   scale/offset：写进张量前做 value = value*scale + offset。
 */
static td_s32 dms_crop_write(const ot_video_frame *frame, const dms_crop_params *p,
                             float *plane0, float *plane1, float *plane2,
                             int out_w, int out_h, size_t stride_bytes,
                             td_bool rgb_order, float scale, float offset)
{
    td_u8 *y_addr = NULL;
    td_u8 *vu_addr = NULL;
    td_u32 y_size = 0;
    td_u32 vu_size = 0;
    td_s32 ret;
    float cos_r;
    float sin_r;
    float rad;
    int ox;
    int oy;

    /*
     * [步骤 1/4] map_cached 只拿虚拟地址；硬件刚写完的缓存必须先 flush，
     * 否则 CPU 可能读到旧像素。两个平面无论后续成功失败都要成对 munmap。
     */
    ret = dms_frame_map(frame, &y_addr, &vu_addr, &y_size, &vu_size);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }

    rad = p->rotate_deg * 3.14159265358979323846f / 180.0f;
    cos_r = cosf(rad);
    sin_r = sinf(rad);

    /* [步骤 2/4] 逐输出像素反算源坐标，而不是把源像素逐个搬过去。 */
    for (oy = 0; oy < out_h; oy++) {
        float dy = ((float)(oy + 0.5f) / (float)out_h - 0.5f) * 2.0f * p->half_h;
        float *row0 = (float *)((td_u8 *)plane0 + (size_t)oy * stride_bytes);
        float *row1 = (float *)((td_u8 *)plane1 + (size_t)oy * stride_bytes);
        float *row2 = (float *)((td_u8 *)plane2 + (size_t)oy * stride_bytes);

        for (ox = 0; ox < out_w; ox++) {
            float dx = ((float)(ox + 0.5f) / (float)out_w - 0.5f) * 2.0f * p->half_w;
            /*
             * roll > 0 表示画面顺时针旋转；舍不得这个符号约定，倾斜的眼睛会往
             * 反方向转，分类器看到的就是倒着的眼。
             */
            float rx = dx * cos_r - dy * sin_r;
            float ry = dx * sin_r + dy * cos_r;
            float sx = p->cx + rx;
            float sy = p->cy + ry;
            float b;
            float g;
            float r;
            float v0;
            float v1;
            float v2;

            /* 双线性采样和 YUV 转色都在这里；源坐标落在图外时采样函数会夹到边界。 */
            dms_sample_nv21(y_addr, vu_addr, frame->stride[0], frame->stride[1],
                            (int)frame->width, (int)frame->height, sx, sy, &b, &g, &r);

            if (rgb_order == TD_TRUE) {
                v0 = r;
                v2 = b;
            } else {
                v0 = b;
                v2 = r;
            }
            v1 = g;
            row0[ox] = v0 * scale + offset;
            row1[ox] = v1 * scale + offset;
            row2[ox] = v2 * scale + offset;
        }
    }

    /* [步骤 3/4] 所有输出像素写完后统一解除缓存映射，防止虚拟地址泄漏。 */
    dms_frame_unmap(y_addr, vu_addr, y_size, vu_size);
    /* [步骤 4/4] 返回成功；三个平面已经按模型要求的通道顺序和归一化参数写好。 */
    return TD_SUCCESS;
}

/*
 * 检测输入快速路径：整帧 640x640 → 640x640 是 1:1，不需要双线性插值。
 * 直接逐像素 NV21→RGB（色度取所在 2x2 块的那一个，标准 NV21 上采样），
 * 写成 (v-127.5)/128。比通用的 dms_crop_write（每像素 3 次双线性 + floorf）
 * 快很多，板端实测 DMS 只有约 3.7fps，这一步是主要的 CPU 开销之一。
 */
static td_s32 dms_write_det_input(const ot_video_frame *frame, float *plane_r, float *plane_g,
                                  float *plane_b, size_t stride_bytes)
{
    td_u8 *y_addr = NULL;
    td_u8 *vu_addr = NULL;
    td_u32 y_size = 0;
    td_u32 vu_size = 0;
    td_u32 x;
    td_u32 y;
    const float k = 1.0f / 128.0f;

    if (dms_frame_map(frame, &y_addr, &vu_addr, &y_size, &vu_size) != TD_SUCCESS) {
        return TD_FAILURE;
    }
    for (y = 0; y < frame->height; y++) {
        const td_u8 *yrow = y_addr + (size_t)y * frame->stride[0];
        const td_u8 *vurow = vu_addr + (size_t)(y / 2) * frame->stride[1];
        float *rr = (float *)((td_u8 *)plane_r + (size_t)y * stride_bytes);
        float *gr = (float *)((td_u8 *)plane_g + (size_t)y * stride_bytes);
        float *br = (float *)((td_u8 *)plane_b + (size_t)y * stride_bytes);

        for (x = 0; x < frame->width; x++) {
            float yv = (float)yrow[x];
            float v = (float)vurow[x & ~1u] - 128.0f;
            float u = (float)vurow[(x & ~1u) + 1] - 128.0f;
            float r = yv + 1.402f * v;
            float g = yv - 0.344136f * u - 0.714136f * v;
            float b = yv + 1.772f * u;

            if (r < 0.0f) { r = 0.0f; } else if (r > 255.0f) { r = 255.0f; }
            if (g < 0.0f) { g = 0.0f; } else if (g > 255.0f) { g = 255.0f; }
            if (b < 0.0f) { b = 0.0f; } else if (b > 255.0f) { b = 255.0f; }
            rr[x] = (r - 127.5f) * k;
            gr[x] = (g - 127.5f) * k;
            br[x] = (b - 127.5f) * k;
        }
    }
    dms_frame_unmap(y_addr, vu_addr, y_size, vu_size);
    return TD_SUCCESS;
}

/* 把连续的 planar 缓冲（plane0/1/2 各 w*h 个 float）按 stride 写进模型输入。 */
static td_s32 dms_put_planar(npu_model *model, const float *buf, int w, int h)
{
    npu_tensor_info info;
    unsigned char *base;
    size_t plane_bytes;
    size_t row_bytes;
    int row;
    td_s32 ret;

    ret = npu_model_get_input_info(model, &info);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }
    if (info.size < 3 * info.stride * (size_t)h) {
        printf("dms_models: input buffer too small (%zu)\n", info.size);
        return TD_FAILURE;
    }
    base = (unsigned char *)npu_model_get_input_buffer(model);
    if (base == NULL) {
        return TD_FAILURE;
    }
    plane_bytes = info.stride * (size_t)h;
    row_bytes = (size_t)w * sizeof(float);
    for (row = 0; row < h; row++) {
        /* 模型输入每行可能比有效像素长，按 stride 分段拷贝可保留填充区。 */
        memcpy(base + (size_t)row * info.stride, buf + (size_t)row * w, row_bytes);
        memcpy(base + plane_bytes + (size_t)row * info.stride,
               buf + (size_t)w * h + (size_t)row * w, row_bytes);
        memcpy(base + 2 * plane_bytes + (size_t)row * info.stride,
               buf + 2 * (size_t)w * h + (size_t)row * w, row_bytes);
    }
    return TD_SUCCESS;
}

/* 核对模型输入尺寸。期望 dims = [1,3,H,W]。 */
static td_s32 dms_check_input(npu_model *model, const char *name, int expect_w, int expect_h)
{
    npu_tensor_info info;
    td_s32 ret;

    ret = npu_model_get_input_info(model, &info);
    if (ret != TD_SUCCESS) {
        printf("dms_models: %s get input info failed\n", name);
        return TD_FAILURE;
    }
    if (info.dim_count != 4 || info.dims[0] != 1 || info.dims[1] != 3 ||
        info.dims[2] != expect_h || info.dims[3] != expect_w) {
        printf("dms_models: %s unexpected input shape %lldx%lldx%lldx%lld\n", name,
               (long long)info.dims[0], (long long)info.dims[1],
               (long long)info.dims[2], (long long)info.dims[3]);
        return TD_FAILURE;
    }
    printf("dms_models: %s input %dx%d ok\n", name, expect_w, expect_h);
    return TD_SUCCESS;
}

/*
 * 输入：调用方的模型对象。
 * 输出：加载 SCRFD、PFLD 和眼睛模型并检查输入形状。
 * 注意：任一模型失败都跳到 fail，先卸载已经成功加载的模型。
 */
td_s32 dms_models_init(dms_models *m)
{
    if (m == NULL) {
        return TD_FAILURE;
    }
    dms_models_reset_pose();
    memset(m, 0, sizeof(*m));

    m->det = npu_model_load(APP_DMS_DET_MODEL_PATH);
    if (m->det == NULL) {
        printf("dms_models: load det failed: %s\n", APP_DMS_DET_MODEL_PATH);
        return TD_FAILURE;
    }
    if (dms_check_input(m->det, "det", DMS_DET_SIZE, DMS_DET_SIZE) != TD_SUCCESS) {
        goto fail;
    }

    m->lm = npu_model_load(APP_DMS_LM_MODEL_PATH);
    if (m->lm == NULL) {
        printf("dms_models: load lm failed: %s\n", APP_DMS_LM_MODEL_PATH);
        goto fail;
    }
    if (dms_check_input(m->lm, "lm", DMS_LM_SIZE, DMS_LM_SIZE) != TD_SUCCESS) {
        goto fail;
    }

    m->eye = npu_model_load(APP_DMS_EYE_MODEL_PATH);
    if (m->eye == NULL) {
        printf("dms_models: load eye failed: %s\n", APP_DMS_EYE_MODEL_PATH);
        goto fail;
    }
    if (dms_check_input(m->eye, "eye", DMS_EYE_SIZE, DMS_EYE_SIZE) != TD_SUCCESS) {
        goto fail;
    }

    /* 头姿不再用 NPU 模型：head-pose-estimation-adas-0001 的 Caffe 权重官方未公开，
     * 现在改用 98 点关键点的 PnP 解算（见 dms_models_run_pose）。
     * m->pose 保留在结构体里但恒为 NULL，deinit 已做 NULL 判断，安全。 */
    m->pose = NULL;

    printf("dms_models: all 3 models loaded\n");
    return TD_SUCCESS;

fail:
    dms_models_deinit(m);
    return TD_FAILURE;
}

/* 输入：模型对象。输出：逐项卸载模型；字段先置 NULL，重复释放也不会二次卸载。 */
void dms_models_deinit(dms_models *m)
{
    if (m == NULL) {
        return;
    }
    if (m->det != NULL) {
        npu_model_unload(m->det);
        m->det = NULL;
    }
    if (m->lm != NULL) {
        npu_model_unload(m->lm);
        m->lm = NULL;
    }
    if (m->eye != NULL) {
        npu_model_unload(m->eye);
        m->eye = NULL;
    }
    if (m->pose != NULL) {
        npu_model_unload(m->pose);
        m->pose = NULL;
    }
}

/*
 * 输入：640x640 NV21 帧和输出框容量。
 * 输出：SCRFD 检测框及数量；先整帧 RGB 归一化，再跑 NPU 和 NMS。
 */
td_s32 dms_models_detect(dms_models *m, const ot_video_frame *frame,
                         dms_box *boxes, td_u32 max_boxes, td_u32 *count)
{
    npu_tensor_info info;
    unsigned char *base;
    size_t plane_bytes;
    td_s32 ret;

    if (m == NULL || m->det == NULL || count == NULL) {
        return TD_FAILURE;
    }
    if (dms_check_frame(frame) != TD_SUCCESS) {
        return TD_FAILURE;
    }
    ret = npu_model_get_input_info(m->det, &info);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }
    base = (unsigned char *)npu_model_get_input_buffer(m->det);
    if (base == NULL) {
        return TD_FAILURE;
    }
    plane_bytes = info.stride * (size_t)APP_DMS_IN_HEIGHT;
    if (info.size < 3 * plane_bytes) {
        printf("dms_models: det input buffer too small (%zu)\n", info.size);
        return TD_FAILURE;
    }

    /* [步骤 2/3] 写 SCRFD 输入：RGB、(v-127.5)/128，与训练时的归一化一致。
     * 整帧 1:1，走快速路径（不做插值）。 */
    ret = dms_write_det_input(frame, (float *)base,
                              (float *)(base + plane_bytes),
                              (float *)(base + 2 * plane_bytes), info.stride);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }

    /* [步骤 3/3] 执行检测并解码 anchor/NMS；输出框坐标仍然属于 640x640。 */
    ret = npu_model_execute(m->det);
    if (ret != TD_SUCCESS) {
        printf("dms_models: det execute failed: %d\n", ret);
        return TD_FAILURE;
    }

    *count = dms_detect_decode(m->det, APP_DMS_SCORE_THRESH, APP_DMS_NMS_THRESH,
                               boxes, max_boxes);
    return TD_SUCCESS;
}

/*
 * 输入：640x640 帧和外扩后的人脸框。
 * 输出：98 点关键点（640x640 坐标）。
 * 注意：PFLD 吃正方形裁剪；输出是归一化坐标，必须乘裁剪边长再加原点。
 */
td_s32 dms_models_run_landmarks(dms_models *m, const ot_video_frame *frame,
                                const dms_box *box, dms_landmarks *lm)
{
    dms_crop_params crop;
    npu_tensor_info out_info;
    const float *out;
    float box_w;
    float box_h;
    float crop_side;
    float crop_x1;
    float crop_y1;
    float max_side;
    int i;
    td_s32 ret;

    if (m == NULL || m->lm == NULL || box == NULL || lm == NULL) {
        return TD_FAILURE;
    }
    if (dms_check_frame(frame) != TD_SUCCESS) {
        return TD_FAILURE;
    }

    box_w = box->x2 - box->x1;
    box_h = box->y2 - box->y1;
    if (box_w <= 0.0f || box_h <= 0.0f) {
        return TD_FAILURE;
    }

    /*
     * PFLD 吃以人脸为中心的正方形裁剪：边长 max(w,h)×1.1（box 必须是检测/跟踪的
     * 原始框，不要再外扩）。人脸始终放在裁剪中心，不做平移钳制——超出画面的部分
     * 由采样函数按边缘像素填充，这样人脸在 112x112 里的位置和大小与训练一致。
     */
    crop_side = (box_w > box_h ? box_w : box_h) * DMS_LM_CROP_MARGIN;
    max_side = 2.0f * (float)APP_DMS_IN_WIDTH;
    if (crop_side > max_side) {
        crop_side = max_side;
    }
    crop_x1 = (box->x1 + box->x2) * 0.5f - crop_side * 0.5f;
    crop_y1 = (box->y1 + box->y2) * 0.5f - crop_side * 0.5f;

    crop.cx = crop_x1 + crop_side * 0.5f;
    crop.cy = crop_y1 + crop_side * 0.5f;
    crop.half_w = crop_side * 0.5f;
    crop.half_h = crop_side * 0.5f;
    crop.rotate_deg = 0.0f;

    /* PFLD 98 点：BGR，像素/255（0~1）。 */
    ret = dms_crop_write(frame, &crop, g_face_buf, g_face_buf + DMS_LM_SIZE * DMS_LM_SIZE,
                         g_face_buf + 2 * DMS_LM_SIZE * DMS_LM_SIZE,
                         DMS_LM_SIZE, DMS_LM_SIZE, DMS_LM_SIZE * sizeof(float),
                         TD_FALSE, 1.0f / 255.0f, 0.0f);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }
    ret = dms_put_planar(m->lm, g_face_buf, DMS_LM_SIZE, DMS_LM_SIZE);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }
    ret = npu_model_execute(m->lm);
    if (ret != TD_SUCCESS) {
        printf("dms_models: lm execute failed: %d\n", ret);
        return TD_FAILURE;
    }

    ret = npu_model_get_output_info(m->lm, 0, &out_info);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }
    if (out_info.size < DMS_LANDMARK_COUNT * 2 * sizeof(float)) {
        printf("dms_models: lm output too small\n");
        return TD_FAILURE;
    }
    out = (const float *)npu_model_get_output_buffer(m->lm, 0);
    if (out == NULL) {
        return TD_FAILURE;
    }

    /* 逐点映射回原图：训练输出 0~1，乘真实裁剪边长再加裁剪左上角。 */
    for (i = 0; i < DMS_LANDMARK_COUNT; i++) {
        /* 输出是相对实际正方形裁剪的归一化坐标（0~1），换算回 640x640 帧坐标。 */
        lm->pt[i].x = out[2 * i] * crop_side + crop_x1;
        lm->pt[i].y = out[2 * i + 1] * crop_side + crop_y1;
    }
    return TD_SUCCESS;
}

/* ======================== 头姿 PnP（纯 C + math.h） ========================
 *
 * 原 head-pose-estimation-adas-0001 的 Caffe 权重官方从未公开，无法转 .om，
 * 因此改用 98 点关键点的 Perspective-n-Point 解算头姿。
 *
 * 2D 点（98 点索引，640x640 帧坐标）：
 *   54  鼻尖
 *   16  下巴（轮廓中点）
 *   97  左瞳孔 / 96 右瞳孔（这里用左右瞳孔，比眼角更稳定、更少受表情影响）
 *   76  左嘴角 / 82 右嘴角
 *
 * 3D 人脸模型点（单位 mm，通用人脸近似值，坐标系用 OpenCV 相机约定
 * X 右 / Y 下 / Z 正方向朝场景（远离相机），与投影公式 u=fX/Z+cx,
 * v=fY/Z+cy 一致）：
 *   鼻尖     (   0,    0,    0 )
 *   下巴     (   0,   65,   20 )
 *   左眼瞳孔 (  31.5, -38,   22 )  （画面右侧，2D idx 97）
 *   右眼瞳孔 ( -31.5, -38,   22 )  （画面左侧，2D idx 96）
 *   左嘴角   ( -25,   27,   18 )  （画面左侧，2D idx 76）
 *   右嘴角   (  25,   27,   18 )  （画面右侧，2D idx 82）
 * 参考比例来自用户给定的 dms_pnp_project 六点模型；左右对应保持与旧表一致。
 * 绝对尺度只影响平移，不影响角度。
 *
 * 相机内参（估算，未标定）：DMS 输入 640x640。VPSS 通道 2 先等比缩放到
 *   1136x640，再裁中间 640x640（CV610 的通道裁剪在缩放之后），等价原图中央
 *   约 1440x1440；像素为正方形，fx≈983.4、fy≈984.9 px，主点 (320, 320)，无畸变。
 *   APP_DMS_CENTER_CROP=0 时为整幅拉伸，fx=554、fy=985。
 *   板端实车标定后应替换为真实内参。
 *
 * 角度正负号约定（OpenCV 相机坐标：X 右 / Y 下 / Z 朝场景，从观察者看画面）：
 *   yaw   > 0：头向右转（鼻尖指向画面右侧）
 *   pitch > 0：抬头（鼻尖指向画面上方）
 *   roll  > 0：头向右倾（画面顺时针，右耳下沉）
 * 旋转矩阵分解为 R = Rx(a)·Ry(b)·Rz(roll)，再换成上面的符号约定：
 *   人脸朝向（鼻尖方向）在模型系是 -Z，相机系里为 -R[:,2]；
 *   a>0 时朝向的 y 分量为 +sin(a)（指向画面下方 = 低头），
 *   b>0 时朝向的 x 分量为 -sin(b)（指向画面左侧），
 *   所以对外输出取反：pitch = -a，yaw = -b，roll 不变：
 *   yaw   = -asin(R[0][2])
 *   roll  = atan2(-R[0][1], R[0][0])
 *   pitch = -atan2(-R[1][2], R[2][2]) = atan2(R[1][2], R[2][2])
 * （2026-11-20 修正：此前未取反，pitch>0 实际是低头，导致 pitch<-20 的
 *   “低头分心”判定实际在抬头时触发、低头时永不触发；已用合成投影验证：
 *   鼻尖朝下 25° 解出 pitch=-25，朝画面右 30° 解出 yaw=+30。）
 */

#define PNP_MAX_ITER    40

static const float g_pnp_model[PNP_POINT_COUNT][3] = {
    {   0.0f,    0.0f,    0.0f },   /* 0 鼻尖  (2D idx 54) */
    {   0.0f,   65.0f,   20.0f },   /* 1 下巴  (2D idx 16) */
    {  31.5f,  -38.0f,   22.0f },   /* 2 左眼瞳孔 (2D idx 97，画面右) */
    { -31.5f,  -38.0f,   22.0f },   /* 3 右眼瞳孔 (2D idx 96，画面左) */
    { -25.0f,   27.0f,   18.0f },   /* 4 左嘴角 (2D idx 76，画面左) */
    {  25.0f,   27.0f,   18.0f },   /* 5 右嘴角 (2D idx 82，画面右) */
};

static const int g_pnp_idx[PNP_POINT_COUNT] = { 54, 16, 97, 96, 76, 82 };

/* 相机内参（估算，未标定）。原图 2560 宽时的估算焦距 2216 px（水平 FOV 约 60°）。 */
#if APP_DMS_CENTER_CROP
/* 等比缩放到 1136x640 再裁 640x640：水平 ×1136/2560、垂直 ×640/1440。
 * 主点 = 缩放后画面中心减去裁剪起点（偏移为 0 时正好 320）。 */
static const float g_pnp_fx = 2216.0f * (float)APP_DMS_SCALED_WIDTH / (float)APP_VIDEO_WIDTH;
static const float g_pnp_fy = 2216.0f * (float)APP_DMS_IN_HEIGHT / (float)APP_VIDEO_HEIGHT;
static const float g_pnp_cx = (float)APP_DMS_SCALED_WIDTH * 0.5f - (float)APP_DMS_CROP_X;
#else
/* 不裁剪：2560x1440 整幅拉伸到 640x640，水平 ×0.25、垂直 ×0.444，fy = fx × 1.778。 */
static const float g_pnp_fx = 554.0f;
static const float g_pnp_fy = 985.0f;
static const float g_pnp_cx = 320.0f;
#endif
static const float g_pnp_cy = 320.0f;

/* 旋转向量（轴角，3 参数）→ 3x3 旋转矩阵（Rodrigues）。 */
static void dms_rotvec_to_mat(const float w[3], float r[9])
{
    float theta = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    float x;
    float y;
    float z;
    float c;
    float s;
    float c1;

    /* 旋转向量接近 0 时 sin/theta 会除零；直接返回单位矩阵。 */
    if (theta < 1e-8f) {
        r[0] = 1.0f; r[1] = 0.0f; r[2] = 0.0f;
        r[3] = 0.0f; r[4] = 1.0f; r[5] = 0.0f;
        r[6] = 0.0f; r[7] = 0.0f; r[8] = 1.0f;
        return;
    }
    x = w[0] / theta;
    y = w[1] / theta;
    z = w[2] / theta;
    c = cosf(theta);
    s = sinf(theta);
    c1 = 1.0f - c;

    r[0] = c + x * x * c1;
    r[1] = x * y * c1 - z * s;
    r[2] = x * z * c1 + y * s;
    r[3] = y * x * c1 + z * s;
    r[4] = c + y * y * c1;
    r[5] = y * z * c1 - x * s;
    r[6] = z * x * c1 - y * s;
    r[7] = z * y * c1 + x * s;
    r[8] = c + z * z * c1;
}

/* 6x6 线性方程组求解（高斯消元 + 部分主元），成功返回 0。 */
static int dms_solve_6x6(float a[PNP_PARAM_COUNT][PNP_PARAM_COUNT],
                         const float b[PNP_PARAM_COUNT], float x[PNP_PARAM_COUNT])
{
    float m[PNP_PARAM_COUNT][PNP_PARAM_COUNT + 1];
    int i;
    int j;
    int k;
    int max_row;
    float temp;
    float pivot;

    /* 每个迭代先选绝对值最大的行作主元，避免小主元把误差放大。 */
    for (i = 0; i < PNP_PARAM_COUNT; i++) {
        for (j = 0; j < PNP_PARAM_COUNT; j++) {
            m[i][j] = a[i][j];
        }
        m[i][PNP_PARAM_COUNT] = b[i];
    }
    for (i = 0; i < PNP_PARAM_COUNT; i++) {
        max_row = i;
        for (k = i + 1; k < PNP_PARAM_COUNT; k++) {
            if (fabsf(m[k][i]) > fabsf(m[max_row][i])) {
                max_row = k;
            }
        }
        /* 主元是 0：方程组奇异，没必要继续算，返回失败让上层回退冷启动。 */
        if (fabsf(m[max_row][i]) < 1e-12f) {
            return -1;
        }
        if (max_row != i) {
            for (j = i; j <= PNP_PARAM_COUNT; j++) {
                temp = m[i][j];
                m[i][j] = m[max_row][j];
                m[max_row][j] = temp;
            }
        }
        pivot = m[i][i];
        for (j = i; j <= PNP_PARAM_COUNT; j++) {
            m[i][j] /= pivot;
        }
        for (k = 0; k < PNP_PARAM_COUNT; k++) {
            if (k == i) {
                continue;
            }
            temp = m[k][i];
            if (temp == 0.0f) {
                continue;
            }
            for (j = i; j <= PNP_PARAM_COUNT; j++) {
                m[k][j] -= temp * m[i][j];
            }
        }
    }
    for (i = 0; i < PNP_PARAM_COUNT; i++) {
        x[i] = m[i][PNP_PARAM_COUNT];
    }
    return 0;
}

/* 投影 + 解析雅可比。
 * 参数 p = [wx,wy,wz, tx,ty,tz]（旋转向量 + 平移）。 */
static void dms_pnp_project(const float p[PNP_PARAM_COUNT], int idx,
                            float *u, float *v,
                            float jac[2][PNP_PARAM_COUNT])
{
    float r[9];
    float px = g_pnp_model[idx][0];
    float py = g_pnp_model[idx][1];
    float pz = g_pnp_model[idx][2];
    float rx;
    float ry;
    float rz;
    float cx;
    float cy;
    float cz;
    float denom;

    dms_rotvec_to_mat(p, r);
    rx = r[0] * px + r[1] * py + r[2] * pz;
    ry = r[3] * px + r[4] * py + r[5] * pz;
    rz = r[6] * px + r[7] * py + r[8] * pz;
    cx = rx + p[3];
    cy = ry + p[4];
    cz = rz + p[5];

    if (fabsf(cz) < 1e-6f) {
        cz = 1e-6f;
    }
    denom = cz;
    *u = g_pnp_fx * cx / denom + g_pnp_cx;
    *v = g_pnp_fy * cy / denom + g_pnp_cy;

    if (jac != NULL) {
        /* 对平移的雅可比。 */
        jac[0][3] = g_pnp_fx / denom;
        jac[0][4] = 0.0f;
        jac[0][5] = -g_pnp_fx * cx / (denom * denom);
        jac[1][3] = 0.0f;
        jac[1][4] = g_pnp_fy / denom;
        jac[1][5] = -g_pnp_fy * cy / (denom * denom);

        /* 对旋转向量的精确雅可比：d(R p)/dw = -[R p]_x · J_l(w)，
         * J_l(w) = I + a·[w]_x + b·[w]_x²，a=(1-cosθ)/θ²，b=(θ-sinθ)/θ³。 */
        {
            float wx = p[0];
            float wy = p[1];
            float wz = p[2];
            float theta = sqrtf(wx * wx + wy * wy + wz * wz);
            float a;
            float b;
            float jl[3][3];
            float dr[3][3];
            float rp[3];
            int row;
            int col;
            int k;

            if (theta < 1e-9f) {
                a = 0.5f;
                b = 1.0f / 6.0f;
            } else {
                a = (1.0f - cosf(theta)) / (theta * theta);
                b = (theta - sinf(theta)) / (theta * theta * theta);
            }
            jl[0][0] = 1.0f + b * (-wy * wy - wz * wz);
            jl[0][1] = a * (-wz) + b * (wx * wy);
            jl[0][2] = a * wy + b * (wx * wz);
            jl[1][0] = a * wz + b * (wx * wy);
            jl[1][1] = 1.0f + b * (-wx * wx - wz * wz);
            jl[1][2] = a * (-wx) + b * (wy * wz);
            jl[2][0] = a * (-wy) + b * (wx * wz);
            jl[2][1] = a * wx + b * (wy * wz);
            jl[2][2] = 1.0f + b * (-wx * wx - wy * wy);

            rp[0] = rx;
            rp[1] = ry;
            rp[2] = rz;
            /* [rp]_x · J_l */
            for (row = 0; row < 3; row++) {
                for (col = 0; col < 3; col++) {
                    float skew_row[3];
                    if (row == 0) {
                        skew_row[0] = 0.0f;     skew_row[1] = -rp[2];  skew_row[2] = rp[1];
                    } else if (row == 1) {
                        skew_row[0] = rp[2];    skew_row[1] = 0.0f;   skew_row[2] = -rp[0];
                    } else {
                        skew_row[0] = -rp[1];   skew_row[1] = rp[0];   skew_row[2] = 0.0f;
                    }
                    dr[row][col] = -(skew_row[0] * jl[0][col] +
                                     skew_row[1] * jl[1][col] +
                                     skew_row[2] * jl[2][col]);
                }
            }
            for (k = 0; k < 3; k++) {
                jac[0][k] = g_pnp_fx * (dr[0][k] * denom - cx * dr[2][k]) / (denom * denom);
                jac[1][k] = g_pnp_fy * (dr[1][k] * denom - cy * dr[2][k]) / (denom * denom);
            }
        }
    }
}

/*
 * 输入：98 点关键点和可选诊断结构。
 * 输出：yaw/pitch/roll（度）；失败时清零角度并报告具体失败阶段。
 * 注意：先用上一帧解作热启动，失败后再用瞳孔连线冷启动；两次都失败才返回失败。
 */
td_s32 dms_models_run_pose(const dms_landmarks *lm,
                           float *yaw, float *pitch, float *roll,
                           dms_pose_debug *debug)
{
    float obs[PNP_POINT_COUNT][2];
    float cold_p[PNP_PARAM_COUNT];
    float p[PNP_PARAM_COUNT];
    float jtj[PNP_PARAM_COUNT][PNP_PARAM_COUNT];
    float rhs[PNP_PARAM_COUNT];
    float delta[PNP_PARAM_COUNT];
    float cost = 0.0f;
    float p_new[PNP_PARAM_COUNT];
    float new_cost = 0.0f;
    float lambda = 1e-3f;
    float pupil_dx;
    float pupil_dy;
    float pupil_dist;
    float initial_roll;
    int idx;
    int i;
    int j;
    int iter = 0;
    int solved;
    int step_finite;
    int attempt;
    int attempt_count;
    float r[9];
    float sy;
    dms_pose_fail_reason fail_reason = DMS_POSE_FAIL_NONE;
    td_s32 result = TD_FAILURE;

    if (debug != NULL) {
        debug->cost = -1.0f;
        debug->iterations = 0;
        debug->reason = DMS_POSE_FAIL_INPUT;
    }

    if (yaw != NULL) {
        *yaw = 0.0f;
    }
    if (pitch != NULL) {
        *pitch = 0.0f;
    }
    if (roll != NULL) {
        *roll = 0.0f;
    }
    if (lm == NULL || yaw == NULL || pitch == NULL || roll == NULL) {
        return TD_FAILURE;
    }

    /* [步骤 1/6] 取六个 2D 观测点；坐标 <=0 视为没有有效关键点。 */
    for (idx = 0; idx < PNP_POINT_COUNT; idx++) {
        obs[idx][0] = lm->pt[g_pnp_idx[idx]].x;
        obs[idx][1] = lm->pt[g_pnp_idx[idx]].y;
        if (!(obs[idx][0] > 0.0f) || !(obs[idx][1] > 0.0f)) {
            fail_reason = DMS_POSE_FAIL_LANDMARK;   /* 关键点坐标非法（0,0 视为未解算出来） */
            goto finish;
        }
    }

    /* [步骤 2/6] 冷启动：用瞳孔间距反推深度，用鼻尖偏移反推 tx/ty。 */
    pupil_dx = lm->pt[97].x - lm->pt[96].x;
    pupil_dy = lm->pt[97].y - lm->pt[96].y;
    pupil_dist = hypotf(pupil_dx, pupil_dy);
    initial_roll = (fabsf(pupil_dx) > 1e-6f) ? atan2f(pupil_dy, pupil_dx) : 0.0f;
    cold_p[0] = 0.0f;
    cold_p[1] = 0.0f;
    cold_p[2] = initial_roll;
    /* 63mm 为模型双瞳间距；瞳孔检测失效时深度用 600mm 兜底。 */
    cold_p[5] = (pupil_dist < 1.0f) ? 600.0f :
                g_pnp_fx * 63.0f / pupil_dist;
    cold_p[3] = (lm->pt[54].x - g_pnp_cx) * cold_p[5] / g_pnp_fx;
    cold_p[4] = (lm->pt[54].y - g_pnp_cy) * cold_p[5] / g_pnp_fy;

    /* [步骤 3/6] 第一次尝试热启动，第二次固定尝试冷启动；最多两轮。 */
    attempt_count = g_has_last_pnp ? 2 : 1;
    for (attempt = 0; attempt < attempt_count; attempt++) {
        if (attempt == 0 && g_has_last_pnp) {
            memcpy(p, g_last_pnp, sizeof(p));
        } else {
            memcpy(p, cold_p, sizeof(p));
        }
        lambda = 1e-3f;
        cost = 0.0f;
        fail_reason = DMS_POSE_FAIL_NONE;

        /* [步骤 4/6] 每轮重算投影、雅可比和误差，再解 6x6 正规方程。 */
        for (iter = 0; iter < PNP_MAX_ITER; iter++) {
            cost = 0.0f;
            memset(jtj, 0, sizeof(jtj));
            memset(rhs, 0, sizeof(rhs));

            for (idx = 0; idx < PNP_POINT_COUNT; idx++) {
                float u;
                float v;
                float jac[2][PNP_PARAM_COUNT];
                float ex;
                float ey;

                dms_pnp_project(p, idx, &u, &v, jac);
                ex = u - obs[idx][0];
                ey = v - obs[idx][1];
                cost += ex * ex + ey * ey;

                for (i = 0; i < PNP_PARAM_COUNT; i++) {
                    rhs[i] += jac[0][i] * ex + jac[1][i] * ey;
                    for (j = 0; j < PNP_PARAM_COUNT; j++) {
                        jtj[i][j] += jac[0][i] * jac[0][j] +
                                     jac[1][i] * jac[1][j];
                    }
                }
            }

            if (cost < 1e-6f) {
                break;
            }
            for (i = 0; i < PNP_PARAM_COUNT; i++) {
                jtj[i][i] += lambda * (jtj[i][i] + 1e-6f);
            }
            solved = dms_solve_6x6(jtj, rhs, delta);
            if (solved != 0) {
                fail_reason = DMS_POSE_FAIL_SOLVE;
                break;
            }
            step_finite = 1;
            for (i = 0; i < PNP_PARAM_COUNT; i++) {
                p_new[i] = p[i] - delta[i];
                if (!isfinite(p_new[i])) {
                    step_finite = 0;
                }
            }

            /* 试探步只算残差，不重复计算雅可比；步长无效按拒绝处理。 */
            new_cost = 0.0f;
            if (step_finite) {
                for (idx = 0; idx < PNP_POINT_COUNT; idx++) {
                    float u;
                    float v;
                    float ex;
                    float ey;

                    dms_pnp_project(p_new, idx, &u, &v, NULL);
                    ex = u - obs[idx][0];
                    ey = v - obs[idx][1];
                    new_cost += ex * ex + ey * ey;
                }
            }

            if (step_finite && isfinite(new_cost) && new_cost < cost) {
                memcpy(p, p_new, sizeof(p));
                cost = new_cost;
                lambda *= 0.3f;
                if (lambda < 1e-9f) {
                    lambda = 1e-9f;
                }
            } else {
                lambda *= 10.0f;
                if (lambda > 1e6f) {
                    if (cost < 20000.0f) {
                        break;
                    }
                    fail_reason = DMS_POSE_FAIL_DIVERGE;
                    break;
                }
            }
        }

        /* [步骤 5/6] 重投影误差过大说明几何关系不成立，按失败处理。 */
        if (fail_reason == DMS_POSE_FAIL_NONE && !(cost < 20000.0f)) {
            fail_reason = DMS_POSE_FAIL_COST;   /* 重投影误差太大，视为解算失败 */
        }
        /* 本轮失败就跳到下一次尝试；没有热启动时 direct finish，由调用方判无效。 */
        if (fail_reason != DMS_POSE_FAIL_NONE) {
            continue;
        }

        /* [步骤 6/6] 把旋转矩阵拆成角度；再用范围门控滤掉镜像/病态解。 */
        dms_rotvec_to_mat(p, r);
        /* R = Rx(a)·Ry(b)·Rz(roll) 的分解；yaw/pitch 取反后才符合
         * “yaw>0 鼻尖朝画面右、pitch>0 抬头”的对外约定（见文件头）。 */
        sy = r[2];
        if (sy > 1.0f) { sy = 1.0f; }
        if (sy < -1.0f) { sy = -1.0f; }
        *yaw = -asinf(sy) * 180.0f / 3.14159265358979323846f;
        *roll = atan2f(-r[1], r[0]) * 180.0f / 3.14159265358979323846f;
        *pitch = atan2f(r[5], r[8]) * 180.0f / 3.14159265358979323846f;

        /* 合理性门控：驾驶员监控场景头姿不会超过这些范围；超出视为解算失败，
         * 避免平面 PnP 的镜像/病态解污染分心判定。 */
        /* 75/60/75 度是驾驶员正常头姿上限；超出通常是 PnP 选了错误镜像解。 */
        if (fabsf(*yaw) > 75.0f || fabsf(*pitch) > 60.0f || fabsf(*roll) > 75.0f) {
            fail_reason = DMS_POSE_FAIL_RANGE;
            continue;
        }

        memcpy(g_last_pnp, p, sizeof(g_last_pnp));
        g_has_last_pnp = TD_TRUE;
        result = TD_SUCCESS;
        break;
    }

finish:
    if (result != TD_SUCCESS) {
        *yaw = 0.0f;
        *pitch = 0.0f;
        *roll = 0.0f;
    }
    if (debug != NULL) {
        debug->cost = cost;
        debug->iterations = iter;
        debug->reason = fail_reason;
    }
    return result;
}

/*
 * 输入：帧、98 点关键点和 roll。
 * 输出：左右眼状态；分类器结果按 out[0] < out[1] 判为睁眼。
 * 注意：debug 非空时同时保留原始两路输出，便于板端核对模型极性。
 */
td_s32 dms_models_run_eye(dms_models *m, const ot_video_frame *frame,
                          const dms_landmarks *lm, float roll,
                          dms_eye_state *left, dms_eye_state *right,
                          dms_eye_debug debug[2])
{
    dms_eye_state states[2];
    int side;
    td_s32 ret;

    if (m == NULL || m->eye == NULL || lm == NULL || left == NULL || right == NULL) {
        return TD_FAILURE;
    }
    if (debug != NULL) {
        memset(debug, 0, 2 * sizeof(debug[0]));
        debug[0].out0 = -1.0f;
        debug[0].out1 = -1.0f;
        debug[1].out0 = -1.0f;
        debug[1].out1 = -1.0f;
    }
    if (dms_check_frame(frame) != TD_SUCCESS) {
        return TD_FAILURE;
    }

    /* side0 = 左眼（*left），side1 = 右眼（*right），与 35 点模型语义一致。
     * 98 点眼组点序（主机 onnxruntime 实测确认）：
     *   右眼 60-67 = 外角60 → 上睑61/62/63 → 内角64 → 下睑65/66/67；
     *   左眼 68-75 = 内角68 → 上睑69/70/71 → 外角72 → 下睑73/74/75。
     * 每只眼用两眼角点定眼框（和原来用眼角间距的做法一致，最稳）：
     *   左眼外角 72 / 内角 68，右眼外角 60 / 内角 64。
     * 眼宽 = 两眼角距离，DMS_EYE_SCALE 逻辑不变。 */
    {
        static const int eye_corner[2][2] = { { 72, 68 },   /* 左眼：外角, 内角 */
                                              { 60, 64 } }; /* 右眼：外角, 内角 */
        for (side = 0; side < 2; side++) {
        const dms_point *p1 = &lm->pt[eye_corner[side][0]];
        const dms_point *p2 = &lm->pt[eye_corner[side][1]];
        dms_crop_params crop;
        npu_tensor_info out_info;
        const float *out;
        float dist;
        float *buf = g_eye_buf + side * 3 * DMS_EYE_SIZE * DMS_EYE_SIZE;

        dist = sqrtf((p2->x - p1->x) * (p2->x - p1->x) +
                     (p2->y - p1->y) * (p2->y - p1->y));
        /* 两眼角几乎重合通常是关键点坏了；沿用 demo 行为当作闭眼。 */
        if (dist <= 1.0f) {
            /* 关键点塌缩：demo 里当作闭眼。 */
            states[side] = DMS_EYE_CLOSED;
            continue;
        }

        /* 眼框中心取两眼角中点，半边长按眼距 × 1.8 留出上下眼皮余量。 */
        crop.cx = (p1->x + p2->x) * 0.5f;
        crop.cy = (p1->y + p2->y) * 0.5f;
        crop.half_w = dist * 0.5f * DMS_EYE_SCALE;
        crop.half_h = crop.half_w;
        /* 按人体 roll 反向采样，把歪头画面“扶正”后再送 32x32 分类器。 */
        crop.rotate_deg = roll;
        if (debug != NULL) {
            debug[side].cx = crop.cx;
            debug[side].cy = crop.cy;
            debug[side].side = crop.half_w * 2.0f;
        }

        /* open-closed-eye-0001：BGR planar，(v-127)/255（官方转换参数）。
         * 对应的 .om 必须用同一量程的校准数据重转（open_closed_eye_m127.om）。 */
        ret = dms_crop_write(frame, &crop, buf, buf + DMS_EYE_SIZE * DMS_EYE_SIZE,
                             buf + 2 * DMS_EYE_SIZE * DMS_EYE_SIZE,
                             DMS_EYE_SIZE, DMS_EYE_SIZE, DMS_EYE_SIZE * sizeof(float),
                             TD_FALSE, 1.0f / DMS_EYE_SCALE_DIV,
                             -DMS_EYE_MEAN / DMS_EYE_SCALE_DIV);
        if (ret != TD_SUCCESS) {
            return TD_FAILURE;
        }
        ret = dms_put_planar(m->eye, buf, DMS_EYE_SIZE, DMS_EYE_SIZE);
        if (ret != TD_SUCCESS) {
            return TD_FAILURE;
        }
        ret = npu_model_execute(m->eye);
        if (ret != TD_SUCCESS) {
            printf("dms_models: eye execute failed: %d\n", ret);
            return TD_FAILURE;
        }
        ret = npu_model_get_output_info(m->eye, 0, &out_info);
        if (ret != TD_SUCCESS ||
            out_info.stride < sizeof(float) ||
            out_info.size < out_info.stride + sizeof(float)) {
            return TD_FAILURE;
        }
        out = (const float *)npu_model_get_output_buffer(m->eye, 0);
        if (out == NULL) {
            return TD_FAILURE;
        }
        {
            const float out0 = *(const float *)out;
            const float out1 = *(const float *)((const char *)out + out_info.stride);

            if (debug != NULL) {
                debug[side].out0 = out0;
                debug[side].out1 = out1;
                debug[side].valid = TD_TRUE;
            }
            /* open-closed-eye-0001 的两个输出按字节 stride 分隔：out0 < out1 为睁眼
             * （与 OpenVINO 官方 demo 一致）。板端用的是去掉 softmax 的 logits 版 .om，
             * 比较大小的结论不变。这里只是单眼分类结果，最终闭眼判定在 dms_metrics.c。 */
            states[side] = (out0 < out1) ? DMS_EYE_OPEN : DMS_EYE_CLOSED;
        }
    }

    }

    *left = states[0];
    *right = states[1];
    return TD_SUCCESS;
}
