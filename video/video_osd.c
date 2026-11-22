/*
 * video_osd.c：RTSP 画面的文字叠加（OSD）。
 * 流水线位置：视频链路的显示末站，位于 VENC 硬件编码输出之前。
 * 输入：DMS 每秒发来的短文本；输出：在 VENC 通道 0 上叠加的 ARGB1555 位图。
 * 调用关系：main.c 初始化/注销，并把 dms_sink 的 overlay 回调接到 video_osd_set_text。
 *
 * APP_DMS_OSD_RGN = 1：用 SDK 的 REGION（RGN）模块在 VENC 通道 0 上画字。
 *   - 区域类型：OVERLAY，像素格式 ARGB_1555，位图尺寸 1024x8（128 字符 x 8x8 点阵）。
 *   - 挂载：ot_mpp_chn = {OT_ID_VENC, 0, 0}，位置 (16,16)，1Hz 刷新位图。
 *   - 文字：代码内置 8x8 点阵字体（见 g_osd_font），只画实际出现的字符。
 *
 * 实现依据（全部对着真实 SDK 头文件/样例核对过，不是凭记忆）：
 *   - smp/a7_linux/source/out/include/ss_mpi_region.h:15-33
 *       ss_mpi_rgn_create / destroy / attach_to_chn / detach_from_chn / set_bmp
 *   - smp/a7_linux/source/out/include/ot_common_region.h
 *       :14-19  ot_rgn_handle（td_u32）
 *       :33-47  ot_rgn_overlay_attr（pixel_format/bg_color/size/canvas_num/clut）
 *       :56-67  ot_rgn_overlay_chn_attr（point/fg_alpha/bg_alpha/layer/qp_info/dst）
 *       :92-95  ot_rgn_attr（type + attr.overlay）
 *       :99-104 ot_rgn_chn_attr（is_show/type + attr.overlay_chn）
 *   - smp/a7_linux/source/out/include/ot_common_video.h:405-410  ot_bmp（pixel_format/width/height/data）
 *   - smp/a7_linux/source/mpp/sample/kol/region/region.c
 *       :33-57   region_create_overlay（canvas_num/pixel_format/size/bg_color + ss_mpi_rgn_create）
 *       :68-95   region_attach_overlay（chn_attr 各字段 + ss_mpi_rgn_attach_to_chn）
 *       :101-119 region_set_bit_map（malloc + ss_mpi_rgn_set_bmp）
 *       :139-159 region_overlay_detach（ss_mpi_rgn_detach_from_chn）
 *       :174-197 region_draw_font（detach -> set_attr -> attach -> set_bmp 的更新顺序）
 *   - smp/a7_linux/source/mpp/sample/common/sample_comm_region.c
 *       :415-441 sample_region_create_overlay、:540-560 sample_region_attach、:588-605 sample_region_set_bit_map
 *
 * 句柄：OVERLAY 的最小句柄是 0（sample_comm_region.c:12 OVERLAY_MIN_HANDLE），
 *       VENC 单通道最多 8 个 OVERLAY（ot_defines.h:224 OT_RGN_VENC_MAX_OVERLAY_NUM），
 *       这里只用句柄 0。项目里没有其他地方用 RGN，不会冲突。
 *
 * 失败策略：RGN 任何一步失败只打印日志并回收已分配资源，函数仍返回 TD_SUCCESS，
 *           文字继续缓存，绝不影响采图/编码/RTSP/录像主流程。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>

#include "app_config.h"
#include "video_osd.h"

#if APP_DMS_OSD_RGN
#include "ss_mpi_region.h"
#endif

/* 缓存的文字上限，比 DMS_OVERLAY_TEXT_MAX(96) 略大，避免截断。 */
#define VIDEO_OSD_TEXT_MAX 128

static char g_osd_text[VIDEO_OSD_TEXT_MAX];
static pthread_mutex_t g_osd_lock = PTHREAD_MUTEX_INITIALIZER;

#if APP_DMS_OSD_RGN
/*
 * 8x8 点阵字体：每行 1 字节，bit7 是最左边的像素（1 = 点亮）。
 * 只覆盖 DMS 叠加文字实际会用到的字符：空格、数字、大写字母、小数点，
 * 另外补了几个常见符号（- _ : /）备用。未收录的字符按空格显示。
 */
#define VIDEO_OSD_FONT_WIDTH  8
#define VIDEO_OSD_FONT_HEIGHT 8

typedef struct {
    char ch;
    td_u8 rows[VIDEO_OSD_FONT_HEIGHT];
} video_osd_glyph;

static const video_osd_glyph g_osd_font[] = {
    { ' ', { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
    { '.', { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30 } },
    { '-', { 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00 } },
    { '_', { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E } },
    { ':', { 0x00, 0x30, 0x30, 0x00, 0x00, 0x30, 0x30, 0x00 } },
    { '/', { 0x02, 0x04, 0x04, 0x08, 0x10, 0x20, 0x20, 0x40 } },
    { '0', { 0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C, 0x00 } },
    { '1', { 0x18, 0x28, 0x08, 0x08, 0x08, 0x08, 0x3C, 0x00 } },
    { '2', { 0x3C, 0x42, 0x02, 0x04, 0x08, 0x10, 0x7E, 0x00 } },
    { '3', { 0x3C, 0x42, 0x02, 0x1C, 0x02, 0x42, 0x3C, 0x00 } },
    { '4', { 0x04, 0x0C, 0x14, 0x24, 0x7E, 0x04, 0x04, 0x00 } },
    { '5', { 0x7E, 0x40, 0x7C, 0x02, 0x02, 0x42, 0x3C, 0x00 } },
    { '6', { 0x3C, 0x40, 0x7C, 0x42, 0x42, 0x42, 0x3C, 0x00 } },
    { '7', { 0x7E, 0x02, 0x04, 0x08, 0x10, 0x10, 0x10, 0x00 } },
    { '8', { 0x3C, 0x42, 0x42, 0x3C, 0x42, 0x42, 0x3C, 0x00 } },
    { '9', { 0x3C, 0x42, 0x42, 0x7E, 0x02, 0x02, 0x3C, 0x00 } },
    { 'A', { 0x18, 0x24, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x00 } },
    { 'B', { 0x7C, 0x42, 0x7C, 0x42, 0x42, 0x42, 0x7C, 0x00 } },
    { 'C', { 0x3C, 0x42, 0x40, 0x40, 0x40, 0x42, 0x3C, 0x00 } },
    { 'D', { 0x78, 0x44, 0x42, 0x42, 0x42, 0x44, 0x78, 0x00 } },
    { 'E', { 0x7E, 0x40, 0x7C, 0x40, 0x40, 0x40, 0x7E, 0x00 } },
    { 'F', { 0x7E, 0x40, 0x7C, 0x40, 0x40, 0x40, 0x40, 0x00 } },
    { 'G', { 0x3C, 0x42, 0x40, 0x4E, 0x42, 0x42, 0x3C, 0x00 } },
    { 'H', { 0x42, 0x42, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x00 } },
    { 'I', { 0x3C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00 } },
    { 'J', { 0x1E, 0x08, 0x08, 0x08, 0x08, 0x48, 0x30, 0x00 } },
    { 'K', { 0x42, 0x44, 0x48, 0x70, 0x48, 0x44, 0x42, 0x00 } },
    { 'L', { 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x7E, 0x00 } },
    { 'M', { 0x42, 0x66, 0x5A, 0x42, 0x42, 0x42, 0x42, 0x00 } },
    { 'N', { 0x42, 0x62, 0x52, 0x4A, 0x46, 0x42, 0x42, 0x00 } },
    { 'O', { 0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C, 0x00 } },
    { 'P', { 0x7C, 0x42, 0x42, 0x7C, 0x40, 0x40, 0x40, 0x00 } },
    { 'Q', { 0x3C, 0x42, 0x42, 0x42, 0x52, 0x4A, 0x3C, 0x02 } },
    { 'R', { 0x7C, 0x42, 0x42, 0x7C, 0x48, 0x44, 0x42, 0x00 } },
    { 'S', { 0x3E, 0x40, 0x40, 0x3C, 0x02, 0x02, 0x7C, 0x00 } },
    { 'T', { 0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00 } },
    { 'U', { 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C, 0x00 } },
    { 'V', { 0x42, 0x42, 0x42, 0x42, 0x42, 0x24, 0x18, 0x00 } },
    { 'W', { 0x42, 0x42, 0x42, 0x42, 0x5A, 0x66, 0x42, 0x00 } },
    { 'X', { 0x42, 0x42, 0x24, 0x18, 0x24, 0x42, 0x42, 0x00 } },
    { 'Y', { 0x42, 0x42, 0x24, 0x18, 0x18, 0x18, 0x18, 0x00 } },
    { 'Z', { 0x7E, 0x02, 0x04, 0x18, 0x20, 0x40, 0x7E, 0x00 } },
};

/* 位图尺寸：宽 = 最大字符数 x 字宽，高 = 字高。region 尺寸必须和位图一致。 */
#define VIDEO_OSD_BMP_WIDTH  (VIDEO_OSD_TEXT_MAX * VIDEO_OSD_FONT_WIDTH)  /* 1024 */
#define VIDEO_OSD_BMP_HEIGHT VIDEO_OSD_FONT_HEIGHT                      /* 8 */

/* OVERLAY 句柄 0（OVERLAY_MIN_HANDLE），VENC 通道 0，显示位置 (16,16)。 */
#define VIDEO_OSD_RGN_HANDLE 0
#define VIDEO_OSD_RGN_POS_X  16
#define VIDEO_OSD_RGN_POS_Y  16
#define VIDEO_OSD_REFRESH_SEC 1

static ot_rgn_handle g_osd_rgn = 0;          /* 当前使用的 OVERLAY 句柄 */
static td_u8 *g_osd_bitmap = NULL;          /* ARGB1555 位图缓冲 */
static ot_bmp g_osd_bmp;                     /* 传给 ss_mpi_rgn_set_bmp 的位图 */
static pthread_t g_osd_thread;
static int g_osd_thread_started = 0;
static int g_osd_rgn_ok = 0;                /* create + attach 都成功才为 1 */
static int g_osd_running = 0;
static int g_osd_dirty = 0;                 /* 有新文字待渲染 */
static char g_osd_snapshot[VIDEO_OSD_TEXT_MAX];

static const video_osd_glyph *video_osd_find_glyph(char ch)
{
    td_u32 i;
    for (i = 0; i < sizeof(g_osd_font) / sizeof(g_osd_font[0]); i++) {
        if (g_osd_font[i].ch == ch) {
            return &g_osd_font[i];
        }
    }
    return NULL;
}

/* 输入：文字字符串。输出：把字模画进全局 ARGB1555 位图；不负责提交给 SDK。 */
/*
 * 把文字渲染进 ARGB1555 位图：背景清零（alpha=0 透明），点亮的像素写 0xFFFF
 * （alpha=1、RGB 全 1 = 不透明白）。位图尺寸固定，文字左对齐，超长截断。
 * 处理顺序：先清空整张透明画布，再逐字符查 8x8 字模，最后逐行展开成像素。
 * 这里把字模的 1-bit 行展开成一个像素点：bit7=最左列，0x80>>col 逐列检查；
 * 0xFFFF 写不透明白色，0x0000 留透明，不透明背景会挡住视频。
 */
static void video_osd_render(const char *text)
{
    td_u32 len;
    td_u32 max_chars = VIDEO_OSD_BMP_WIDTH / VIDEO_OSD_FONT_WIDTH;
    td_u32 i, row, col;

    if (text == NULL) {
        return;
    }
    len = (td_u32)strlen(text);
    if (len > max_chars) {
        len = max_chars;
    }

    /* [步骤 1/3] 每个像素 2 字节；清成 0x0000 等于全透明，未写字的地方露出视频。 */
    memset(g_osd_bitmap, 0, (size_t)VIDEO_OSD_BMP_WIDTH * VIDEO_OSD_BMP_HEIGHT * 2);

    /* [步骤 2/3] 每个字符横向占 8 列；先找到字模，没收录的字符留空。 */
    for (i = 0; i < len; i++) {
        const video_osd_glyph *glyph = video_osd_find_glyph(text[i]);
        td_u32 base_x;
        if (glyph == NULL) {
            continue;   /* 未收录的字符留空 */
        }
        base_x = i * VIDEO_OSD_FONT_WIDTH;
        for (row = 0; row < VIDEO_OSD_FONT_HEIGHT; row++) {
            td_u8 bits = glyph->rows[row];
            if (bits == 0) {
                continue;
            }
            /* [步骤 3/3] bit7 对应最左列，所以从 0x80 逐位右移；写 0xFFFF 才是不透明白。 */
            for (col = 0; col < VIDEO_OSD_FONT_WIDTH; col++) {
                if ((bits & (0x80 >> col)) != 0) {
                    td_u32 offset = (row * VIDEO_OSD_BMP_WIDTH + base_x + col) * 2;
                    td_u16 *px = (td_u16 *)(void *)(g_osd_bitmap + offset);
                    *px = 0xFFFF;
                }
            }
        }
    }
}

/* 1Hz 刷新线程：只在文字变化时重渲染并 set_bmp，避免无谓的总线开销。 */
/* 输入：未使用的线程参数。输出：线程退出时返回 NULL；退出前必须由 deinit join。 */
static void *video_osd_refresh_thread(void *arg)
{
    (void)arg;

    while (g_osd_running) {
        struct timespec ts;
        int need_render = 0;

        ts.tv_sec = VIDEO_OSD_REFRESH_SEC;
        ts.tv_nsec = 0;
        nanosleep(&ts, NULL);
        if (!g_osd_running) {
            break;
        }
        if (!g_osd_rgn_ok) {
            continue;
        }

        /* 只在主线程说有新文字时才拷快照，避免锁住每 1 秒的采集/编码。 */
        pthread_mutex_lock(&g_osd_lock);
        if (g_osd_dirty) {
            memcpy(g_osd_snapshot, g_osd_text, sizeof(g_osd_snapshot));
            g_osd_dirty = 0;
            need_render = 1;
        }
        pthread_mutex_unlock(&g_osd_lock);

        if (need_render) {
            td_s32 ret;
            video_osd_render(g_osd_snapshot);
            g_osd_bmp.data = g_osd_bitmap;
            ret = ss_mpi_rgn_set_bmp(g_osd_rgn, &g_osd_bmp);
            if (ret != TD_SUCCESS) {
                printf("video_osd: ss_mpi_rgn_set_bmp failed with %#x!\n", ret);
            }
        }
    }
    return NULL;
}

/* 输入：无。输出：摘掉并销毁当前 RGN；只做资源回收，不关闭视频主流程。 */
static void video_osd_rgn_release(void)
{
    ot_mpp_chn chn;

    if (!g_osd_rgn_ok) {
        return;
    }
    chn.mod_id = OT_ID_VENC;
    chn.dev_id = 0;
    chn.chn_id = 0;
    (void)ss_mpi_rgn_detach_from_chn(g_osd_rgn, &chn);
    (void)ss_mpi_rgn_destroy(g_osd_rgn);
    g_osd_rgn_ok = 0;
}
#endif /* APP_DMS_OSD_RGN */

/*
 * 输入：无。
 * 输出：创建 RGN 位图并挂到 VENC 通道 0；初始化失败只禁用叠加，不阻断主视频。
 * 注意：清理顺序必须与初始化相反，先 detach/destroy RGN，再释放位图内存。
 */
td_s32 video_osd_init(void)
{
#if APP_DMS_OSD_RGN
    ot_rgn_attr region;
    ot_rgn_chn_attr chn_attr;
    ot_mpp_chn chn;
    td_s32 ret;

    memset(&g_osd_bmp, 0, sizeof(g_osd_bmp));
    memset(g_osd_snapshot, 0, sizeof(g_osd_snapshot));

    g_osd_bitmap = (td_u8 *)calloc(1, (size_t)VIDEO_OSD_BMP_WIDTH * VIDEO_OSD_BMP_HEIGHT * 2);
    if (g_osd_bitmap == NULL) {
        printf("video_osd: bitmap alloc failed\n");
        return TD_SUCCESS;   /* 只影响叠加，主流程继续 */
    }

    memset(&region, 0, sizeof(region));
    region.type = OT_RGN_OVERLAY;
    region.attr.overlay.pixel_format = OT_PIXEL_FORMAT_ARGB_1555;
    region.attr.overlay.size.width = VIDEO_OSD_BMP_WIDTH;
    region.attr.overlay.size.height = VIDEO_OSD_BMP_HEIGHT;
    region.attr.overlay.bg_color = 0x0000;   /* 透明背景 */
    region.attr.overlay.canvas_num = 1;

    /* OVERLAY 最小句柄是 0；SDK 只允许这个句柄用于本项目这一块文字区。 */
    ret = ss_mpi_rgn_create(VIDEO_OSD_RGN_HANDLE, &region);
    if (ret != TD_SUCCESS) {
        printf("video_osd: ss_mpi_rgn_create failed with %#x!\n", ret);
        free(g_osd_bitmap);
        g_osd_bitmap = NULL;
        return TD_SUCCESS;
    }
    g_osd_rgn = VIDEO_OSD_RGN_HANDLE;

    memset(&chn, 0, sizeof(chn));
    chn.mod_id = OT_ID_VENC;
    chn.dev_id = 0;
    chn.chn_id = 0;

    memset(&chn_attr, 0, sizeof(chn_attr));
    chn_attr.is_show = TD_TRUE;
    chn_attr.type = OT_RGN_OVERLAY;
    chn_attr.attr.overlay_chn.point.x = VIDEO_OSD_RGN_POS_X;
    chn_attr.attr.overlay_chn.point.y = VIDEO_OSD_RGN_POS_Y;
    chn_attr.attr.overlay_chn.fg_alpha = 255;
    chn_attr.attr.overlay_chn.bg_alpha = 255;
    chn_attr.attr.overlay_chn.layer = 0;
    chn_attr.attr.overlay_chn.qp_info.enable = TD_TRUE;
    chn_attr.attr.overlay_chn.qp_info.is_abs_qp = TD_FALSE;
    chn_attr.attr.overlay_chn.qp_info.qp_val = 0;
    chn_attr.attr.overlay_chn.dst = OT_RGN_ATTACH_JPEG_MAIN;

    ret = ss_mpi_rgn_attach_to_chn(g_osd_rgn, &chn, &chn_attr);
    if (ret != TD_SUCCESS) {
        printf("video_osd: ss_mpi_rgn_attach_to_chn failed with %#x!\n", ret);
        (void)ss_mpi_rgn_destroy(g_osd_rgn);
        free(g_osd_bitmap);
        g_osd_bitmap = NULL;
        return TD_SUCCESS;
    }
    g_osd_rgn_ok = 1;

    g_osd_bmp.pixel_format = OT_PIXEL_FORMAT_ARGB_1555;
    g_osd_bmp.width = VIDEO_OSD_BMP_WIDTH;
    g_osd_bmp.height = VIDEO_OSD_BMP_HEIGHT;

    g_osd_running = 1;
    if (pthread_create(&g_osd_thread, NULL, video_osd_refresh_thread, NULL) != 0) {
        printf("video_osd: refresh thread create failed\n");
        g_osd_running = 0;
        video_osd_rgn_release();
        free(g_osd_bitmap);
        g_osd_bitmap = NULL;
        return TD_SUCCESS;
    }
    g_osd_thread_started = 1;

    printf("video_osd: RGN overlay on VENC chn0 ready (%ux%u bitmap)\n",
        (td_u32)VIDEO_OSD_BMP_WIDTH, (td_u32)VIDEO_OSD_BMP_HEIGHT);
#else
    pthread_mutex_lock(&g_osd_lock);
    g_osd_text[0] = '\0';
    pthread_mutex_unlock(&g_osd_lock);
#endif
    return TD_SUCCESS;
}

/* 输入：要显示的文字及未使用的回调参数。输出：更新缓存并置脏标志，等待刷新线程重画。 */
void video_osd_set_text(const char *text, void *arg)
{
    (void)arg;   /* 钩子签名统一带 arg，OSD 用不到 */

    if (text == NULL) {
        return;
    }

    pthread_mutex_lock(&g_osd_lock);
    snprintf(g_osd_text, sizeof(g_osd_text), "%s", text);
#if APP_DMS_OSD_RGN
    /* 只更新缓存并置脏标志，实际重画由 1Hz 线程去做。 */
    g_osd_dirty = 1;
#endif
    pthread_mutex_unlock(&g_osd_lock);
}

/* 输入：无。输出：停刷新线程、摘掉并销毁 RGN、释放位图并清空缓存文字。 */
void video_osd_deinit(void)
{
#if APP_DMS_OSD_RGN
    /* 顺序与 init 相反：先停线程，再 detach + destroy，最后释放位图。 */
    if (g_osd_thread_started) {
        g_osd_running = 0;
        pthread_join(g_osd_thread, NULL);
        g_osd_thread_started = 0;
    }
    video_osd_rgn_release();
    free(g_osd_bitmap);
    g_osd_bitmap = NULL;
#endif
    pthread_mutex_lock(&g_osd_lock);
    g_osd_text[0] = '\0';
    pthread_mutex_unlock(&g_osd_lock);
}
