/*
 * dms_occlusion.c：dms_occlusion.h 的实现（从 dms_sink.c 拆出）。
 * 流水线位置：DMS 线程每隔 DMS_OCCLUSION_INTERVAL 帧调用一次。
 * 输入：VPSS 通道 2 的一帧画面。输出：疑似遮挡时打印一行日志（暂未接入报警）。
 */
#include <stdio.h>
#include <stddef.h>

#include "app_config.h"
#include "dms_occlusion.h"
#include "ss_mpi_sys.h"
#include "ss_mpi_sys_mem.h"

/* ---------------- 摄像头遮挡自检（占位） ---------------- */

/*
 * TODO：法规要求摄像头遮挡/失效检测。这里先给一个基于亮度方差的占位实现：
 * 画面被遮挡（镜头被挡住、严重欠曝）时 Y 的方差会很小。
 * 实车标定时要改阈值，并考虑和 ISP 的 AE 状态联合判断，避免误报。
 */
void dms_occlusion_check(const ot_video_frame *frame)
{
    td_u8 *y_addr = NULL;
    td_u32 y_size;
    td_u64 sum = 0;
    td_u64 sum_sq = 0;
    td_u32 count = 0;
    td_u32 row;
    td_u32 col;
    double mean;
    double var;

    if (frame == NULL || frame->width == 0 || frame->height == 0) {
        return;
    }
    y_size = frame->stride[0] * frame->height;
    y_addr = (td_u8 *)ss_mpi_sys_mmap_cached(frame->phys_addr[0], y_size);
    if (y_addr == NULL) {
        return;
    }
    if (ss_mpi_sys_flush_cache(frame->phys_addr[0], y_addr, y_size) != TD_SUCCESS) {
        (void)ss_mpi_sys_munmap(y_addr, y_size);
        return;
    }

    /* 每隔 8 个像素采一个样，够判断"是不是被挡住"即可。 */
    for (row = 0; row < frame->height; row += 8) {
        const td_u8 *line = y_addr + (size_t)row * frame->stride[0];
        for (col = 0; col < frame->width; col += 8) {
            td_u32 v = line[col];
            sum += v;
            sum_sq += (td_u64)v * v;
            count++;
        }
    }
    (void)ss_mpi_sys_munmap(y_addr, y_size);

    if (count == 0) {
        return;
    }
    mean = (double)sum / (double)count;
    var = (double)sum_sq / (double)count - mean * mean;
    if (var < 0.0) {
        var = 0.0;
    }
    if (var < (double)APP_DMS_OCCLUSION_VAR_THRESH) {
        printf("dms: camera occlusion suspected (y_var=%.1f)\n", var);
        /*
         * TODO：这里应该产生一个 DMS_EVENT_OCCLUSION 事件并走分级报警。
         * 现在只打印，不接入报警状态机，避免误报影响主功能。
         */
    }
}
