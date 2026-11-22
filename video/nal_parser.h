/*
 * 【说明】nal_parser：理解"H.264 码流长什么样"，并把里面的关键信息拿出来。
 *
 *   【H.264 码流的结构（务必理解）】
 *   编码器输出的一帧不是一整块，而是由若干个 NAL 单元（Network Abstraction Layer Unit）首尾相连：
 *      00 00 00 01 | 67 ... | 00 00 00 01 | 68 ... | 00 00 00 01 | 65 ... (大量数据)
 *      └起始码┘     └SPS ┘    └起始码┘     └PPS ┘    └起始码┘     └IDR 关键帧切片┘
 *   - 起始码：00 00 01 或 00 00 00 01，用来标记"下一个 NAL 从这里开始"
 *   - 起始码后的第 1 个字节叫 NAL 头；它的低 5 位是 nal_unit_type（类型）：
 *         1 = 普通 P 帧切片     5 = IDR 关键帧切片
 *         7 = SPS（序列参数集：分辨率、profile、level）
 *         8 = PPS（图像参数集：熵编码方式、量化参数）
 *   - SPS/PPS 是解码器的"说明书"，没有它们就无法解码。但编码器只在关键帧附近输出它们，
 *     所以我们要把它们缓存起来，客户端 DESCRIBE 时放进 SDP 的 sprop-parameter-sets 里。
 */

/**
 * @file nal_parser.h
 * @brief H.264码流的NAL单元解析，以及SPS/PPS缓存。
 */
#ifndef NAL_PARSER_H
#define NAL_PARSER_H

#include "ot_type.h"

/* 一帧里最多记录的NAL单元数，正常远小于这个值。 */
#define NAL_MAX_COUNT 64

/* SPS/PPS的长度上限。 */
#define NAL_SPS_MAX_SIZE 256
#define NAL_PPS_MAX_SIZE 64

/*
 * 【说明】nal_info 只记录"位置信息"——偏移、长度、类型，不拷贝数据，数据仍在原来的帧缓冲里。
 */
/* 一个NAL单元在帧内的位置描述。 */
typedef struct {
    td_u32 offset;   /* 从帧首开始的偏移，指向NAL头（起始码之后） */
    td_u32 len;      /* NAL长度，含1字节NAL头，不含起始码 */
    td_u8  type;     /* nal_unit_type：1普通帧 5关键帧 7SPS 8PPS */
} nal_info;

/* 一帧的解析结果。 */
typedef struct {
    nal_info nals[NAL_MAX_COUNT];
    td_u32   count;       /* 实际NAL个数 */
    td_u32   total;       /* 帧总字节数 */
    td_bool  truncated;   /* NAL个数超过上限时为真 */
} nal_frame_info;

/* 解析一帧码流，记录每个NAL的位置、长度和类型。 */
td_s32 nal_parse_frame(const td_u8 *data, td_u32 len, nal_frame_info *info);

/* 从一帧码流中提取SPS/PPS并缓存，供RTSP的DESCRIBE使用。 */
void nal_cache_sps_pps(const td_u8 *data, td_u32 len);

/* 取一份缓存的SPS/PPS拷贝，尚未捕获到则返回失败。 */
td_s32 nal_get_sps_pps(td_u8 *sps, td_u32 *sps_len, td_u8 *pps, td_u32 *pps_len);

/* 标准Base64编码，输出以'\0'结尾。 */
/*
 * 【说明】Base64：把任意二进制字节转成"只含 A-Z a-z 0-9 + /"的文本，便于放进 SDP 这类文本协议。
 *   规则：每 3 字节(24 位)切成 4 组 6 位，每组映射成一个字符；不足 3 字节的用 '=' 补齐。
 *   例：SPS 15 字节 → 20 个字符；PPS 4 字节 → 8 个字符（末尾有 =）。
 */
td_s32 nal_base64_encode(const td_u8 *in, td_u32 in_len, char *out, td_u32 out_size);

#endif
