/*
 * @file nal_parser.c
 * @brief 解析H.264码流中的NAL单元，并缓存SPS/PPS。
 *
 * 一帧码流由若干NAL单元首尾相接组成，每个NAL单元前面有一个起始码
 * （0x000001或0x00000001）。本模块找出每个NAL的位置和类型，并记住
 * 最近一次出现的SPS(type 7)和PPS(type 8)。
 */

#include <stdio.h>

#include "nal_parser.h"

#include <string.h>
#include <pthread.h>

/* H.264 NAL单元类型。类型值位于NAL头字节的低5位。 */

/*
 * 【说明】#define 宏常量：给"魔数"起名字，代码里写 NAL_TYPE_SPS 比写 7 容易读。
 */
#define NAL_TYPE_IDR 5  // IDR 关键帧，解码可从此帧开始
#define NAL_TYPE_SPS 7  // 序列参数集，包含分辨率、编码格式等信息
#define NAL_TYPE_PPS 8  // 图像参数集，包含熵编码、量化等参数

/* 缓存的SPS/PPS，取流线程写、命令线程读，用锁保护。 */
/*
 * 【说明】SPS/PPS 缓存：
 *   写入方是编码取流线程（nal_cache_sps_pps），读取方是 RTSP 客户端线程（nal_get_sps_pps），
 *   是两个不同线程访问同一份数据，所以一定要用 g_sps_lock 保护，保证读到的 SPS 与它的长度是一致的。
 */
static td_u8 g_sps[NAL_SPS_MAX_SIZE];
static td_u32 g_sps_len = 0;
static td_u8 g_pps[NAL_PPS_MAX_SIZE];
static td_u32 g_pps_len = 0;
/* 保护SPS/PPS缓存及其长度，避免读写线程同时访问导致数据不一致。 */
static pthread_mutex_t g_sps_lock = PTHREAD_MUTEX_INITIALIZER;

/*
 * 在data[from, len)中查找下一个起始码，返回起始码与NAL头的位置。
 * 支持H.264常见的3字节起始码0x000001。
 */
/*
 * 【说明】在数据里找下一个起始码 00 00 01。
 *   循环条件 i+3<len：保证读取 data[i+2] 时不越界（越界读是 C 语言最常见的 bug 来源之一）。
 *   4 字节起始码 00 00 00 01 也能被识别：它的后 3 个字节正好是 00 00 01，
 *   前面多出来的 00 会在后面的"去掉末尾填充零"步骤中被丢掉。
 *   返回值通过指针参数（sc_pos / nal_pos）"带出来"——C 语言函数只能 return 一个值，
 *   想返回多个值时，就让调用者传入变量的地址，函数往里面写。
 */
static td_bool nal_find_start_code(const td_u8 *data, td_u32 len, td_u32 from,
                                   td_u32 *sc_pos, td_u32 *nal_pos)
{
    td_s32 i;
    /* 遍历寻找data[from, len)中的第一个起始码0x000001。 */
    /*
     * 【说明】i 是有符号 int，len 是无符号 u32，直接比较会触发 gcc 的符号不一致警告。
     *   i 从 from(>=0) 开始递增，i+3 一定非负，所以先强转成 td_u32 再和 len 比较是安全的。
     */
    for(i=from;(td_u32)(i+3)<len;i++)
    {
        if(data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1)
        {
            if(sc_pos!=NULL)
            {
                *sc_pos = i;
            }
            if(nal_pos!=NULL)
            {
                *nal_pos = i + 3;
            }
            return TD_TRUE;
        }
    }
    return TD_FALSE;
}

/*
 * 解析一帧码流，记录每个NAL单元的位置、长度和类型。
 * info中的offset指向NAL头，不包含前面的起始码。
 */
/*
 * 【说明】★核心解析函数★ 思路：用"起始码"当路标把一帧切成若干段：
 *    1. 找到第一个起始码，NAL 从它后面开始
 *    2. 再找下一个起始码：它就是当前 NAL 的结尾；找不到，说明当前 NAL 一直到帧末尾
 *    3. 去掉结尾多余的 00（可能是 4 字节起始码的第一个 0 或编码器的填充）
 *    4. 记下 offset / len / type（type = NAL 头字节 & 0x1F，只取低 5 位）
 *    5. 回到第 2 步，继续找下一个
 *   输出放在调用者给的 info 里。最多记 NAL_MAX_COUNT=64 个，超过标记 truncated。
 */
td_s32 nal_parse_frame(const td_u8 *data, td_u32 len, nal_frame_info *info)
{
    td_u32 sc_pos = 0;
    td_u32 nal_pos = 0;
    td_s32 ret;

    if (data == NULL || info == NULL)
    {
        printf("Failed to parse NAL: invalid argument\n");
        return TD_FAILURE;
    }

    /* 清空输出结构体，并记录整帧码流长度。 */
    memset(info, 0, sizeof(nal_frame_info));
    info->total = len;

    if(len < 4)
    {
        printf("Failed to parse NAL: frame too short\n");
        return TD_FAILURE;
    }

    /* 查找第一个NAL单元的起始码。 */
    ret=nal_find_start_code(data, len, 0, &sc_pos, &nal_pos);
    if(ret != TD_TRUE)
    {
        printf("Failed to parse NAL: no start code found\n");
        return TD_FAILURE;
    }

    /* 从第一个起始码开始，依次解析每个NAL单元。 */
    while(nal_pos < len)
    {
        td_u32 next_sc = 0;
        td_u32 next_nal = 0;
        td_u32 end;
        td_bool has_next;

        if(info->count >= NAL_MAX_COUNT)
        {
            info->truncated = TD_TRUE;
            break;
        }

        /* 找下一个NAL的起点，找不到说明这是最后一个NAL */
        has_next = nal_find_start_code(data, len, nal_pos, &next_sc, &next_nal);
        if(has_next == TD_TRUE)
        {
            end = next_sc;
        }
        else
        {
            end = len;
        }

        /*
         * 去掉NAL末尾的填充零字节。
         * H.264允许在NAL单元末尾出现trailing_zero_8bits。
         */
        while(nal_pos < end && data[end - 1] == 0)
        {
            end--;
        }

        /* 记录当前NAL单元的偏移、长度和类型。 */
        if(end > nal_pos)
        {
            info->nals[info->count].offset = nal_pos;
            info->nals[info->count].len = end - nal_pos;
            /*
             * 【说明】& 0x1F：按位与，0x1F 的二进制是 0001 1111，只保留低 5 位。
             *   例：NAL 头字节 0x67 = 0110 0111 → & 0x1F = 0000 0111 = 7 → SPS。
             *       NAL 头字节 0x65 = 0110 0101 → & 0x1F = 5 → IDR。
             *   高 3 位含 forbidden_zero_bit(1位) 和 nal_ref_idc(2位，表示重要程度)，这里不关心。
             */
            info->nals[info->count].type = data[nal_pos] & 0x1F; /* nal_unit_type在NAL头的低5位 */
            info->count++;
        }

        if(has_next ==  TD_FALSE)
        {
            break;
        }

        nal_pos = next_nal;
    }

    if(info->count == 0)
    {
        printf("Failed to parse NAL: no valid NAL units found\n");
        return TD_FAILURE;
    }

    return TD_SUCCESS;

}

/*
 * 从一帧码流中提取SPS/PPS并缓存，供DESCRIBE生成SDP使用。
 * 新发现的SPS/PPS会覆盖之前缓存的同类型参数。
 */
/*
 * 【说明】每一帧（含 P 帧）都会调用它，但只有含 SPS/PPS 的帧才会真的更新缓存。
 *   开销很小：只做一次起始码扫描。长度超限的会被忽略，防止写爆缓存数组。
 *   缓存里存的是不含起始码的 NAL 本体（以 67/68 开头），这正是 RTP 和 SDP 想要的格式。
 */
void nal_cache_sps_pps(const td_u8 *data, td_u32 len)
{
    nal_frame_info info;
    td_u32 i;

    /* 先解析一帧码流，获取所有NAL单元的信息。 */
    if(nal_parse_frame(data, len, &info) != TD_SUCCESS)
    {
        printf("Failed to cache SPS/PPS: parse frame failed\n");
        return;
    }
    /* 加锁后访问和更新全局SPS/PPS缓存。 */
    pthread_mutex_lock(&g_sps_lock);
    /* 遍历所有NAL单元，提取SPS和PPS。 */
    for(i = 0; i < info.count; i++)
    {
        if(info.nals[i].type == NAL_TYPE_SPS)
        {
            /* 缓存SPS。 */
            if (info.nals[i].len > NAL_SPS_MAX_SIZE) {
                printf("SPS too long, ignored: %u bytes\n", info.nals[i].len);
                continue;
            }
            memcpy(g_sps, data + info.nals[i].offset, info.nals[i].len);
            g_sps_len = info.nals[i].len;
        }
        else if(info.nals[i].type == NAL_TYPE_PPS)
        {
            /* 缓存PPS。 */
            if(info.nals[i].len > NAL_PPS_MAX_SIZE)
            {
                printf("Failed to cache PPS: length exceeds max size\n");
                continue;
            }
            memcpy(g_pps, data + info.nals[i].offset, info.nals[i].len);
            g_pps_len = info.nals[i].len;
        }
    }
    pthread_mutex_unlock(&g_sps_lock);
}

/* 线程安全地复制当前缓存的SPS/PPS，供调用方生成或发送会话描述。 */
/*
 * 【说明】给 DESCRIBE 用：把缓存的 SPS/PPS 复制给调用者。
 *   返回 FAILURE 的典型原因：程序刚启动，还没出现过第一个 IDR 帧（最多等 2 秒）。
 *   对应 rtsp_server.c 里返回 "503 Service Unavailable" 的情况，客户端过一会重试即可。
 *   复制的是副本，所以调用者拿到手之后即使缓存被更新也不受影响。
 */
td_s32 nal_get_sps_pps(td_u8 *sps, td_u32 *sps_len, td_u8 *pps, td_u32 *pps_len)
{
    if (sps == NULL || sps_len == NULL || pps == NULL || pps_len == NULL) {
        printf("Failed to get SPS/PPS: invalid argument\n");
        return TD_FAILURE;
    }

    /* 加锁，保证读取缓存数据和长度时保持一致。 */
    pthread_mutex_lock(&g_sps_lock);

    /* SPS或PPS任意一个未缓存，都无法返回完整参数集。 */
    if (g_sps_len == 0 || g_pps_len == 0) {
        pthread_mutex_unlock(&g_sps_lock);
        return TD_FAILURE;
    }

    memcpy(sps, g_sps, g_sps_len);
    *sps_len = g_sps_len;

    memcpy(pps, g_pps, g_pps_len);
    *pps_len = g_pps_len;

    pthread_mutex_unlock(&g_sps_lock);

    return TD_SUCCESS;
}

/* 标准Base64编码，用于SDP里的sprop-parameter-sets。 */
td_s32 nal_base64_encode(const td_u8 *in, td_u32 in_len, char *out, td_u32 out_size)
{
    static const char table[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    td_u32 need;
    td_u32 i;
    td_u32 o = 0;

    if (in == NULL || out == NULL) {
        printf("Failed to base64 encode: invalid argument\n");
        return TD_FAILURE;
    }

    /* 1. 检查输出缓冲区：每3字节变4个字符，末尾还有一个'\0' */
    need = ((in_len + 2) / 3) * 4 + 1;
    if (out_size < need) {
        printf("Failed to base64 encode: buffer too small (%u < %u)\n", out_size, need);
        return TD_FAILURE;
    }

    /* 2. 每3字节一组，编码成4个字符，不足部分用'='补齐 */
    for (i = 0; i < in_len; i += 3) {
        td_u32 v = (td_u32)in[i] << 16;

        if (i + 1 < in_len) {
            v |= (td_u32)in[i + 1] << 8;
        }
        if (i + 2 < in_len) {
            v |= (td_u32)in[i + 2];
        }

        out[o++] = table[(v >> 18) & 0x3F];
        out[o++] = table[(v >> 12) & 0x3F];
        out[o++] = (i + 1 < in_len) ? table[(v >> 6) & 0x3F] : '=';
        out[o++] = (i + 2 < in_len) ? table[v & 0x3F] : '=';
    }

    out[o] = '\0';

    return TD_SUCCESS;
}
