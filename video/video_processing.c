/*
 * 【说明】本文件的函数：
 *   （函数已按"实际使用顺序"排列：先启动、再绑定、中间是调试函数、最后是解绑和停止）
 *   video_processing_start()      创建 Group → 启动 Group → 配置 Channel 属性 → 使能 Channel
 *   video_processing_bind_vi()    用 ss_mpi_sys_bind 把 VI 通道接到 VPSS 组
 *   video_processing_unbind_vi()  解除绑定（必须在 stop 之前）
 *   video_processing_stop()       禁用 Channel → 停止 Group → 销毁 Group
 *   test_frame / save_yuv         调试函数，用法和 video_capture.c 里的同名函数几乎一样，
 *                                 只是从 VPSS 通道取帧（取到的是 1280x720 的图）
 */

/*
 * @file video_processing.c
 * @brief 实现VI到VPSS的连接、缩放输出及帧保存。
 */
#include <stdio.h>

#include "app_config.h"
#include "video_processing.h"

#include "ss_mpi_vpss.h"
#include "ss_mpi_sys_bind.h"

#include "ss_mpi_sys_mem.h"

/* 创建、配置并启动 VPSS Group 0 和 Channel 0。 */
/*
 * 【说明】创建顺序：  create_grp → start_grp → set_chn_attr → enable_chn
 *   出错时倒着撤销（stop_grp、destroy_grp 标签）。
 */
td_s32 video_processing_start(void)
{
    td_s32 ret;
    td_s32 cleanup_ret;
/*  Group 属性描述输入图像的格式、尺寸上限等。
    Channel 属性描述这个出口输出多大的图像、什么格式。*/
    ot_vpss_grp vpss_grp = 0;
    ot_vpss_chn vpss_chn = 0;

    /*Group属性配置*/

    /*
     * 【说明】Group 属性（输入端怎么看待进来的图像）：
     *   - max_width / max_height ：这个组最多能处理多大的输入图像（2560x1440）
     *   - pixel_format NV21 / dynamic_range SDR8 ：输入图像格式，要与 VI 通道的输出一致
     *   - ie_en(图像增强) / dci_en(动态对比度) / mcf_en(多光谱融合)：高级功能，这里都关闭
     *   - dei_mode OFF ：DEI = 去隔行(De-Interlace)。摄像头是逐行扫描，不需要
     *   - buf_share_en ：是否让多个通道共享缓存，不使用
     *   - frame_rate -1 ：不做帧率转换
     */
    ot_vpss_grp_attr grp_attr = {
        .ie_en = TD_FALSE,
        .dci_en = TD_FALSE,
        .buf_share_en = TD_FALSE,
        .mcf_en = TD_FALSE,

        .max_width = APP_VIDEO_WIDTH,
        .max_height = APP_VIDEO_HEIGHT,

        .max_dei_width = 0,
        .max_dei_height = 0,

        .dynamic_range = OT_DYNAMIC_RANGE_SDR8,
        .pixel_format = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420,

        /* 当前摄像头输出逐行图像，不需要去隔行 */
        .dei_mode = OT_VPSS_DEI_MODE_OFF,

        .buf_share_chn = 0,

        .frame_rate = {
            .src_frame_rate = -1,
            .dst_frame_rate = -1
        }
    };

    /*
     * 【说明】Channel 属性（输出口输出什么样的图像）：
     *   - width / height   ：1280x720，缩放到这个尺寸（由 VPSS 硬件完成，几乎不占 CPU）
     *   - chn_mode USER    ：由用户指定输出尺寸（另一种是 AUTO 自动）
     *   - depth = 1        ：给应用程序保留 1 帧的"主动取帧队列"，
     *                        只有 depth>0 才能调用 ss_mpi_vpss_get_chn_frame（test_frame/save_yuv 用）。
     *                        正常运行时，帧通过 bind 自动流入 VENC，不需要这个队列，
     *                        它会多占 1 个 1280x720 缓存块；pool1 总共只有 4 块，要留意够不够用。
     *   - border_en / aspect_ratio ：加黑边、保持宽高比，这里不用
     *   - 2560x1440 与 1280x720 宽高比都是 16:9，所以缩放不会变形。
     */
    ot_vpss_chn_attr chn_attr = {
        .mirror_en = TD_FALSE,
        .flip_en = TD_FALSE,
        .border_en = TD_FALSE,

        .width = APP_VPSS_OUT_WIDTH,
        .height = APP_VPSS_OUT_HEIGHT,

        /* 为后续应用主动获取VPSS输出帧保留队列 */
        .depth = 1,

        /* 输出尺寸由应用指定 */
        .chn_mode = OT_VPSS_CHN_MODE_USER,

        .video_format = OT_VIDEO_FORMAT_LINEAR,
        .dynamic_range = OT_DYNAMIC_RANGE_SDR8,
        .pixel_format = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420,
        .compress_mode = OT_COMPRESS_MODE_NONE,

        .frame_rate = {
            .src_frame_rate = -1,
            .dst_frame_rate = -1
        },

        /* 边框和宽高比属性不使用 */
        .aspect_ratio = {
            .mode = OT_ASPECT_RATIO_NONE
        }
    };

    /* 1. 创建VPSS Group */
    /*
     * 【说明】"create → start"是两步：create 只是分配并登记，start 才真正开始接收数据。
     */
    ret = ss_mpi_vpss_create_grp(vpss_grp, &grp_attr);
    if (ret != TD_SUCCESS) {
        printf("Failed to create VPSS Group: %#x\n",
               (td_u32)ret);
        return ret;
    }

    /* 启动处理组。 */
    ret = ss_mpi_vpss_start_grp(vpss_grp);
    if (ret != TD_SUCCESS) {
        printf("Failed to start VPSS Group: %#x\n",
               (td_u32)ret);
        goto destroy_grp;
    }

    /* 配置输出通道。 */
    ret = ss_mpi_vpss_set_chn_attr(vpss_grp, vpss_chn, &chn_attr);
    if (ret != TD_SUCCESS) {
        printf("Failed to set VPSS Channel Attribute: %#x\n",
               (td_u32)ret);
         goto stop_grp;
    }

    /* 4. 使能输出通道。 */
    ret = ss_mpi_vpss_enable_chn(vpss_grp, vpss_chn);
    if (ret != TD_SUCCESS) {
        printf("Enable VPSS channel failed: %#x\n", (td_u32)ret);
        goto stop_grp;
    }

    printf("VPSS Group 0 Channel 0 started, output=%u x %u\n",
           chn_attr.width, chn_attr.height);

    /*
     * 通道 2：给 DMS（驾驶员监控）用的 640x640 画面。
     * 复制通道 0 的属性再改尺寸/帧率。
     * 通道 1 不使用（原来给旧人脸识别，已删除）；DMS 保留在已经板端验证过的通道 2，
     * 不挪到通道 1，避免不同通道的缩放/裁剪能力差异带来新问题。
     */
    ot_vpss_chn dms_chn = APP_DMS_VPSS_CHN;
    ot_vpss_chn_attr dms_chn_attr = chn_attr;

#if APP_DMS_CENTER_CROP
    /* CV610 的通道裁剪发生在缩放之后：先等比缩放到 1136x640（正方形像素），
     * 下面再裁中间 640x640 作为实际输出。 */
    dms_chn_attr.width = APP_DMS_SCALED_WIDTH;
#else
    dms_chn_attr.width = APP_DMS_IN_WIDTH;
#endif
    dms_chn_attr.height = APP_DMS_IN_HEIGHT;
    /* depth = 1：用户队列只留最新的 1 帧，DMS 要的是"现在"的画面。 */
    dms_chn_attr.depth = 1;
    dms_chn_attr.frame_rate.src_frame_rate = APP_VIDEO_FPS;
    dms_chn_attr.frame_rate.dst_frame_rate = APP_DMS_FPS;

    ret = ss_mpi_vpss_set_chn_attr(vpss_grp, dms_chn, &dms_chn_attr);
    if (ret != TD_SUCCESS) {
        printf("Failed to set VPSS Channel 2 attribute: %#x\n", (td_u32)ret);
        goto disable_chn0;
    }

#if APP_DMS_CENTER_CROP
    /* 只对通道 2 启用裁剪。板端实测：CV610 的通道裁剪坐标是“缩放后的通道
     * 输出坐标”（之前按原图写 x=560,w=1440，被钳成了 80x640）。所以这里在
     * 1136x640 的等比缩放画面里裁中间 640x640，等价原图中央约 1440x1440。 */
    {
        ot_vpss_crop_info dms_crop_info;

        dms_crop_info.enable = TD_TRUE;
        dms_crop_info.crop_mode = OT_COORD_ABS;
        dms_crop_info.crop_rect.x = APP_DMS_CROP_X;
        dms_crop_info.crop_rect.y = 0;
        dms_crop_info.crop_rect.width = APP_DMS_IN_WIDTH;
        dms_crop_info.crop_rect.height = APP_DMS_IN_HEIGHT;

        ret = ss_mpi_vpss_set_chn_crop(vpss_grp, dms_chn, &dms_crop_info);
        if (ret != TD_SUCCESS) {
            /* 裁剪失败不能拖垮主码流：退回 640x640 整幅拉伸，DMS 照常出图
             * （此时 PnP 内参不匹配，头姿角度会偏，日志里会看到这条报错）。 */
            printf("Failed to set VPSS Channel 2 crop: %#x, fall back to stretched "
                   "640x640 (head pose angles inaccurate)\n", (td_u32)ret);
            dms_chn_attr.width = APP_DMS_IN_WIDTH;
            ret = ss_mpi_vpss_set_chn_attr(vpss_grp, dms_chn, &dms_chn_attr);
            if (ret != TD_SUCCESS) {
                printf("Failed to reset VPSS Channel 2 attribute: %#x\n", (td_u32)ret);
                goto disable_chn0;
            }
        } else {
            printf("VPSS Channel 2 crop: scale %ux%u then crop x=%d y=0 %ux%u\n",
                   (td_u32)APP_DMS_SCALED_WIDTH, (td_u32)APP_DMS_IN_HEIGHT,
                   (int)APP_DMS_CROP_X, (td_u32)APP_DMS_IN_WIDTH, (td_u32)APP_DMS_IN_HEIGHT);
        }
    }
#else
    printf("VPSS Channel 2 crop: disabled (full frame stretched to %ux%u)\n",
           (td_u32)APP_DMS_IN_WIDTH, (td_u32)APP_DMS_IN_HEIGHT);
#endif

    ret = ss_mpi_vpss_enable_chn(vpss_grp, dms_chn);
    if (ret != TD_SUCCESS) {
        printf("Enable VPSS channel 2 failed: %#x\n", (td_u32)ret);
        goto disable_chn0;
    }

    printf("VPSS Group 0 Channel 2 started, output=%u x %u @ %d fps\n",
           dms_chn_attr.width, dms_chn_attr.height, APP_DMS_FPS);
    return TD_SUCCESS;

disable_chn0:
    cleanup_ret = ss_mpi_vpss_disable_chn(vpss_grp, vpss_chn);
    if (cleanup_ret != TD_SUCCESS) {
        printf("Cleanup: disable VPSS channel 0 failed: %#x\n", (td_u32)cleanup_ret);
    }

stop_grp:
    cleanup_ret = ss_mpi_vpss_stop_grp(vpss_grp);
    if (cleanup_ret != TD_SUCCESS) {
        printf("Cleanup: stop VPSS group failed: %#x\n",
               (td_u32)cleanup_ret);
    }

destroy_grp:
    cleanup_ret = ss_mpi_vpss_destroy_grp(vpss_grp);
    if (cleanup_ret != TD_SUCCESS) {
        printf("Cleanup: destroy VPSS group failed: %#x\n",
               (td_u32)cleanup_ret);
    }

    /* 保留最初失败操作的返回值 */
    return ret;
}

/* 将 VI Pipe 0 Channel 0 连接到 VPSS Group 0。 */
/*
 * 【说明】ot_mpp_chn：用"三元组"描述一个媒体通道的位置：
 *     mod_id  ：属于哪个模块（OT_ID_VI / OT_ID_VPSS / OT_ID_VENC ……）
 *     dev_id  ：设备号（VI 里是 Pipe 号，VPSS 里是 Group 号，VENC 里为 0）
 *     chn_id  ：通道号
 *   ss_mpi_sys_bind(源, 目的) ：建立 源→目的 的数据通路，
 *   从此 VI 的 Chn0 每产生一帧，就自动送入 VPSS Grp0。
 *   ⚠ 我们是 VI 在线(ONLINE)+VPSS 离线(OFFLINE)模式（见 media_system.c），
 *   VI 到 VPSS 实际是通过内存块传递的，bind 仍然需要。
 */
td_s32 video_processing_bind_vi(void)
{
    td_s32 ret;
    ot_mpp_chn src_chn = {
        .mod_id = OT_ID_VI,
        .dev_id = 0,
        .chn_id = 0
    };
    ot_mpp_chn dst_chn = {
        .mod_id = OT_ID_VPSS,
        .dev_id = 0,
        .chn_id = 0
    };

    ret = ss_mpi_sys_bind(&src_chn, &dst_chn);
    if (ret != TD_SUCCESS) {
        printf("Failed to bind VI to VPSS: %#x\n", (td_u32)ret);
        return ret;
    }

    printf("VI Pipe 0 Channel 0 bound to VPSS Group 0 Channel 0\n");
    return TD_SUCCESS;
}

/* 获取并释放一帧，验证 VPSS 输出尺寸。 */
/*
 * 【说明】调试函数：从 VPSS 取一帧，核对尺寸是否是 1280x720，然后还回去。
 *   ss_mpi_vpss_get_chn_frame / release_chn_frame 是"借—还"一对，必须配对。
 *   注意即使尺寸不对（result = FAILURE），也要先把帧还回去再返回，否则内存块被永远占用。
 */
td_s32 video_processing_test_frame(void)
{
    td_s32 ret;
    td_s32 result = TD_SUCCESS;

    ot_vpss_grp vpss_grp = 0;
    ot_vpss_chn vpss_chn = 0;

    /* 保存SDK返回的帧描述信息 */
    ot_video_frame_info frame_info = {0};

    printf("Waiting for a frame from VPSS Group 0 Channel 0...\n");

    /* 1. 从VPSS输出通道获取一帧，最多等待3000毫秒 */
    ret = ss_mpi_vpss_get_chn_frame(
        vpss_grp,
        vpss_chn,
        &frame_info,
        3000);

    if (ret != TD_SUCCESS) {
        printf("Get VPSS frame failed: %#x\n", (td_u32)ret);
        return ret;
    }

    /* 2. 获取成功后，读取这一帧的信息 */
    printf("Get VPSS frame succeeded\n");

    printf("  size: %u x %u\n",
           frame_info.video_frame.width,
           frame_info.video_frame.height);

    printf("  stride: Y=%u, VU=%u\n",
           frame_info.video_frame.stride[0],
           frame_info.video_frame.stride[1]);

    printf("  pixel format: %d\n",
           (int)frame_info.video_frame.pixel_format);

    printf("  time_ref: %u, pts: %llu\n",
           frame_info.video_frame.time_ref,
           (unsigned long long)frame_info.video_frame.pts);

    /* 3. 检查实际尺寸是否符合配置 */
    if (frame_info.video_frame.width != APP_VPSS_OUT_WIDTH ||
        frame_info.video_frame.height != APP_VPSS_OUT_HEIGHT) {
        printf("Unexpected VPSS output size, expected %u x %u\n",
               (td_u32)APP_VPSS_OUT_WIDTH,
               (td_u32)APP_VPSS_OUT_HEIGHT);

        /*
         * 即使尺寸不符合预期，也不能直接返回。
         * 已获取的帧仍然需要释放。
         */
        result = TD_FAILURE;
    }

    /* 4. 使用结束，把帧交还给VPSS */
    ret = ss_mpi_vpss_release_chn_frame(
        vpss_grp,
        vpss_chn,
        &frame_info);

    if (ret != TD_SUCCESS) {
        printf("Release VPSS frame failed: %#x\n", (td_u32)ret);
        return ret;
    }

    printf("VPSS frame released\n");
    return result;
}

/* 从 VPSS 通道获取一帧，保存为无压缩的 NV21 文件。 */
/*
 * 【说明】调试函数：把 VPSS 输出的一帧保存成 NV21 文件。
 *   流程和 video_capture_save_yuv 完全一样（取帧 → mmap → 按行写 → munmap → 还帧），
 *   可参考 video_capture.c 里更详细的说明。
 *   ⚠ 这里的几处 printf 提示还写着 "VI frame"（复制粘贴遗留），实际取的是 VPSS 帧，不影响功能。
 */
td_s32 video_processing_save_yuv(ot_vpss_chn vpss_chn,const char *file_path)
{
    td_s32 ret;
    td_s32 cleanup_ret;
    ot_vpss_grp vpss_grp = 0;

    ot_video_frame_info frame_info = {0};
    const ot_video_frame *frame = &frame_info.video_frame;

    FILE *fp = NULL;
    td_u8 *y_addr = NULL;
    td_u8 *vu_addr = NULL;

    td_u32 y_size = 0;
    td_u32 vu_size = 0;
    td_u32 row;

    if (file_path == NULL) {
        printf("YUV file path is NULL\n");
        return TD_FAILURE;
    }

    /* 1. 获取一帧，最多等待3000毫秒 */
    ret = ss_mpi_vpss_get_chn_frame(
        vpss_grp, vpss_chn, &frame_info, 3000);
    if (ret != TD_SUCCESS) {
        printf("Get VI frame failed: %#x\n", (td_u32)ret);
        return ret;
    }

    /*
     * 从这里开始，无论后续是否成功，
     * 都必须经过cleanup归还已经获取的VI帧。
     */
    ret = TD_FAILURE;

    /* 2. 当前函数只处理8bit、线性、无压缩的NV21 */
    if (frame->pixel_format != OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420 ||
        frame->compress_mode != OT_COMPRESS_MODE_NONE ||
        frame->video_format != OT_VIDEO_FORMAT_LINEAR ||
        frame->dynamic_range != OT_DYNAMIC_RANGE_SDR8) {
        printf("Unsupported frame format for NV21 saving\n");
        goto cleanup;
    }

    /* NV21的宽高应为偶数，行跨度应能够容纳有效像素 */
    if (frame->width == 0 || frame->height == 0 ||
        (frame->width % 2) != 0 || (frame->height % 2) != 0 ||
        frame->stride[0] < frame->width ||
        frame->stride[1] < frame->width) {
        printf("Invalid frame size or stride\n");
        goto cleanup;
    }

    /*
     * 3. 计算两个平面在内存中的长度。
     * 映射范围需要包含每行可能存在的填充数据。
     */
    y_size = frame->stride[0] * frame->height;
    vu_size = frame->stride[1] * (frame->height / 2);

    /*
     * 4. 将物理地址映射为本进程可以访问的虚拟地址。
     * 分别使用SDK提供的Y、VU地址，不假设两个平面连续。
     */
    y_addr = (td_u8 *)ss_mpi_sys_mmap(
        frame->phys_addr[0], y_size);
    if (y_addr == NULL) {
        printf("Map Y plane failed\n");
        goto cleanup;
    }

    vu_addr = (td_u8 *)ss_mpi_sys_mmap(
        frame->phys_addr[1], vu_size);
    if (vu_addr == NULL) {
        printf("Map VU plane failed\n");
        goto cleanup;
    }

    /* 5. 以二进制方式创建文件；同名文件会被覆盖 */
    fp = fopen(file_path, "wb");
    if (fp == NULL) {
        perror("Open YUV file failed");
        goto cleanup;
    }

    /*
     * 6. 先写Y平面。
     * 每行只写width字节，跳过可能存在的行尾填充。
     */
    for (row = 0; row < frame->height; row++) {
        if (fwrite(y_addr + row * frame->stride[0],
                   1, frame->width, fp) != frame->width) {
            printf("Write Y plane failed\n");
            goto cleanup;
        }
    }

    /*
     * 7. 再写VU平面。
     * NV21的VU平面有height/2行，每行有效数据为width字节。
     */
    for (row = 0; row < frame->height / 2; row++) {
        if (fwrite(vu_addr + row * frame->stride[1],
                   1, frame->width, fp) != frame->width) {
            printf("Write VU plane failed\n");
            goto cleanup;
        }
    }

    ret = TD_SUCCESS;

cleanup:
    /* 关闭文件，同时检查缓冲数据是否成功写出 */
    if (fp != NULL) {
        if (fclose(fp) != 0) {
            perror("Close YUV file failed");
            ret = TD_FAILURE;
        }
    }

    /* 解除本函数建立的内存映射 */
    if (vu_addr != NULL) {
        cleanup_ret = ss_mpi_sys_munmap(vu_addr, vu_size);
        if (cleanup_ret != TD_SUCCESS) {
            printf("Unmap VU plane failed: %#x\n",
                   (td_u32)cleanup_ret);
            ret = TD_FAILURE;
        }
    }

    if (y_addr != NULL) {
        cleanup_ret = ss_mpi_sys_munmap(y_addr, y_size);
        if (cleanup_ret != TD_SUCCESS) {
            printf("Unmap Y plane failed: %#x\n",
                   (td_u32)cleanup_ret);
            ret = TD_FAILURE;
        }
    }

    /* 无论保存是否成功，都归还已获取的VI帧 */
    cleanup_ret = ss_mpi_vpss_release_chn_frame(
        vpss_grp, vpss_chn, &frame_info);
    if (cleanup_ret != TD_SUCCESS) {
        printf("Release VI frame failed: %#x\n",
               (td_u32)cleanup_ret);
        ret = TD_FAILURE;
    }

    if (ret == TD_SUCCESS) {
        printf("Saved NV21 frame: %s, %u x %u\n",
               file_path, frame->width, frame->height);
    }

    return ret;
}

/* 解除 VI 到 VPSS 的连接。 */
/*
 * 【说明】ss_mpi_sys_unbind：解除绑定。必须在 VPSS 停止之前执行。
 */
void video_processing_unbind_vi(void)
{
    td_s32 ret;
    ot_mpp_chn src_chn = {
        .mod_id = OT_ID_VI,
        .dev_id = 0,
        .chn_id = 0
    };
    ot_mpp_chn dst_chn = {
        .mod_id = OT_ID_VPSS,
        .dev_id = 0,
        .chn_id = 0
    };

    ret = ss_mpi_sys_unbind(&src_chn, &dst_chn);
    if (ret != TD_SUCCESS) {
        printf("Failed to unbind VI from VPSS: %#x\n", (td_u32)ret);
        return;
    }

    printf("VI Pipe 0 Channel 0 unbound from VPSS Group 0 Channel 0\n");
}

/* 禁用VPSS输出通道，停止并销毁VPSS处理组。 */
/*
 * 【说明】停止顺序：先禁用通道，再停组，最后销毁组（与创建顺序相反）。
 *   注意：调用它之前必须先 unbind（解除 VI 的连接），否则数据还在往里送。
 *   main.c 的 unbind_vpss → stop_vpss 标签就是这个顺序。
 */
void video_processing_stop(void)
{
    td_s32 ret;
    ot_vpss_grp vpss_grp = 0;
    ot_vpss_chn vpss_chn = 0;
    ot_vpss_chn dms_chn = APP_DMS_VPSS_CHN;

    /* 1. 禁用输出通道 */
    ret = ss_mpi_vpss_disable_chn(vpss_grp, dms_chn);
    if (ret != TD_SUCCESS) {
        printf("Failed to disable VPSS channel: %#x\n",
               (td_u32)ret);
    }


    /* 1. 禁用输出通道 */
    ret = ss_mpi_vpss_disable_chn(vpss_grp, vpss_chn);
    if (ret != TD_SUCCESS) {
        printf("Failed to disable VPSS channel: %#x\n",
               (td_u32)ret);
        return;
    }

    /* 2. 停止处理组 */
    ret = ss_mpi_vpss_stop_grp(vpss_grp);
    if (ret != TD_SUCCESS) {
        printf("Failed to stop VPSS group: %#x\n",
               (td_u32)ret);
        return;
    }

    /* 3. 销毁处理组 */
    ret = ss_mpi_vpss_destroy_grp(vpss_grp);
    if (ret != TD_SUCCESS) {
        printf("Failed to destroy VPSS group: %#x\n",
               (td_u32)ret);
        return;
    }

    printf("VPSS Group 0 Channel 0 stopped and destroyed\n");
}
