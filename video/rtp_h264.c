#include "rtp_h264.h"
#include <string.h>

/// RTP 固定头部格式
/*
 * 【说明】手工填写 12 字节 RTP 头（按位对照 rtp_h264.h 里的图）：
 *    buf[0] = 0x80          二进制 10 0 0 0000：V=2（版本2）、P=0、X=0、CC=0
 *    buf[1] = M | PT        最高位是 marker，低 7 位是载荷类型 96
 *    buf[2..3]  序列号 seq（16 位）
 *    buf[4..7]  时间戳 ts（32 位）
 *    buf[8..11] SSRC（32 位）
 *   ★为什么要手工一个字节一个字节地写？★
 *   网络协议规定多字节整数用"大端序"(高字节在前，network byte order)，
 *   而我们的 ARM CPU 是小端序（低字节在前）。直接把 uint16/uint32 变量 memcpy 进去，字节顺序就反了。
 *   所以用  (x >> 8) & 0xff  取高字节、 x & 0xff  取低字节，手工按大端排列。
 *   >>（右移）把数字往低位移动；& 0xff 只保留最低 8 位。
 */
static void rtp_fill_header(uint8_t *buf, int marker, uint16_t seq, uint32_t ts, uint32_t ssrc)
{
    buf[0] = 0x80; /* 版本号2，P=0，X=0，CC=0 */
    buf[1] = (marker ? 0x80 : 0x00) | RTP_PAYLOAD_TYPE96; /* M位和负载类型 */
    buf[2] = (seq >> 8) & 0xff; /* 序列号高字节 */
    buf[3] = seq & 0xff;        /* 序列号低字节 */
    buf[4] = (ts >> 24) & 0xff; /* 时间戳高字节 */
    buf[5] = (ts >> 16) & 0xff;
    buf[6] = (ts >> 8) & 0xff;
    buf[7] = ts & 0xff;        /* 时间戳低字节 */
    buf[8] = (ssrc >> 24) & 0xff; /* SSRC 高字节 */
    buf[9] = (ssrc >> 16) & 0xff;
    buf[10] = (ssrc >> 8) & 0xff;
    buf[11] = ssrc & 0xff;     /* SSRC 低字节 */
}

/*初始化rtp_h264_ctx结构体*/
/*
 * 【说明】初始化上下文。seq 从 0 开始（RFC 建议起始序号随机，但对播放器没有影响）。
 */
void rtp_h264_init(rtp_h264_ctx *ctx, uint32_t ssrc, rtp_send_cb send, void *arg)
{
    ctx->seq = 0;
    ctx->send = send;
    ctx->ssrc = ssrc;
    ctx->arg = arg;
}

/*
 * 【说明】★本文件核心★ 分两种情况：
 *    (A) len <= 1400：Single NAL Unit 包
 *          RTP 包 = [12B RTP头][NAL 原样]
 *          NAL 头本身就是载荷的第一个字节，接收端一看就知道类型。
 *    (B) len > 1400：FU-A 分片（Fragmentation Unit）
 *          RTP 包 = [12B RTP头][FU indicator 1B][FU header 1B][这一片数据]
 *          把 NAL 的第 1 个字节（NAL 头）拿掉，剩余数据切片，
 *          每片前面加两个字节让接收端能还原：
 *           FU indicator = 原NAL头的高3位(F,NRI) | 28    → 28 表示"这是 FU-A"
 *           FU header    = S(起始片)|E(结束片)|R(保留0)|原NAL类型(低5位)
 *          接收端收齐后，用 [FU indicator高3位 + FU header低5位] 重新拼出原来的 NAL 头。
 *   回调 send 返回非 0（发送失败）就立即停止，因为缺一片的 NAL 接收端也拼不出来。
 */
int rtp_h264_send_nal(rtp_h264_ctx *ctx, const uint8_t *nal, size_t len, uint32_t ts,int marker)
{
    /*组装 RTP 包用的临时缓冲区，最终发出去的就是它的内容。*/
    /*
     * pkt：组装 RTP 包用的临时缓冲区，最终发出去的就是它的内容。
     * 内存布局：
     *   | 12 字节 RTP 头 | 载荷（NAL 数据，最多 1400 字节） |
     *   ^pkt             ^pkt + RTP_HEADER_SIZE
     * 它是函数里的局部数组，在栈上，函数返回就失效；
     * 所以回调 send 必须在函数返回前把数据发走或拷走，不能保存 pkt 的指针。
     */
    /*
     * 【说明】局部数组在"栈"上，1412 字节，函数返回就失效。
     *   所以 send 回调必须在返回前把数据发出去或拷走，不能保存这个指针（这就是前面讨论过的"回调不用担心失效"的原因：
     *   UDP 的 sendto 是同步调用，返回时数据已经交给内核了）。
     */
    uint8_t rtp_packet[RPT_MAX_PAYLOAD + RTP_HEADER_SIZE];
    int ret;

    /* 参数检查：任何一个为空，或 NAL 长度为 0，都没有可发的内容 */
    if (ctx == NULL || ctx->send == NULL || nal == NULL || len == 0) {
        return -1;
    }

    if(len<=RPT_MAX_PAYLOAD)
    {
        /* 往 pkt 的前 12 字节写 RTP 头：*/
        rtp_fill_header(rtp_packet, marker, ctx->seq, ts, ctx->ssrc);

        /* 把 NAL 原样拷到头后面，作为载荷（不含 00 00 00 01 起始码） */
        memcpy(rtp_packet + RTP_HEADER_SIZE, nal, len);

        /* 3. 调用回调函数发送 RTP 包 */
        ret = ctx->send(rtp_packet, len + RTP_HEADER_SIZE, ctx->arg);
        if (ret != 0) {
            return ret;
        }

        /* 头里已经用了当前序号，下一个包改用新序号 */
        ctx->seq++;
        return 0;
    }
    else
    {
        /* NAL 数据过大，超过单包载荷上限，用FU-A分片发送 */
        /*将NAL换成FU-A分片*/
        uint8_t nal_header = nal[0]; /* NAL 头，1 字节 */
        /*
         * FU indicator：保留原 NAL 头的高 3 位（F 和 NRI，表示重要程度），
         * 低 5 位改成 28，告诉接收端"这是 FU-A 分片"。
         */
        uint8_t fu_indicator = (nal_header & 0xE0) | 28; /* FU-A 指示符，NAL 类型为 28 */

        uint8_t nal_type = nal_header & 0x1F; /* 原 NAL 类型，低 5 位 */

        const uint8_t *p = nal + 1; /* 指向 NAL 数据的起始位置，跳过 NAL 头 */
        size_t remain = len-1; /* 剩余数据长度，不含 NAL 头 */

        /* 每片数据最多 1400 - 2 字节，因为载荷里要先放 FU indicator 和 FU header */
        /*
         * 【说明】每个分片最多 1400-2 = 1398 字节数据（因为 2 字节要给 FU indicator 和 FU header）。
         *   例：一个 10000 字节的 IDR NAL → 去掉 NAL 头 9999 字节 → 切成 8 片（7 片 1398 + 1 片 213）→ 共 8 个 RTP 包。
         *   第 1 片 S=1；最后一片 E=1 且 M=marker；中间的 S=E=0、M=0。
         */
        const size_t max_frag = RPT_MAX_PAYLOAD - 2;

        int first = 1;   /* 是否是第一片 */

        while (remain > 0) {
            size_t frag; /* 这一片的数据长度 */
            /* 这一片的数据长度：剩余够多就切满，不够就把剩下的全部装进去 */
            if(remain>max_frag)
            {
                frag=max_frag;
            }
            else
            {
                frag=remain;
            }

            /* 这一片装完就没剩了，说明它是最后一片 */
            int last = (frag == remain);

            /*
             * FU header：S(1位) E(1位) R(1位，固定0) Type(5位)
             *   第一片 S=1 -> 0x80
             *   最后一片 E=1 -> 0x40
             *   中间片 S=E=0
             */
            uint8_t fu_header = nal_type;

            if (first) {
                fu_header |= 0x80;
            }
            if (last) {
                fu_header |= 0x40;
            }

            /*
             * 写 RTP 头。marker 只有在"这个 NAL 的最后一片"时才继承调用者传的值，
             * 前面的分片一律为 0。调用者只有在这是本帧最后一个 NAL 时才会传 1。
             */
            if(last)
            {
                /*
                 * 【说明】M 位（marker）只能出现在"最后一个包"。这里分片时只有最后一片继承调用者给的 marker，
                 *   前面的分片一律为 0；而调用者（rtp_sender）只在"这一帧的最后一个 NAL"才传 1。
                 *   所以一帧里 M=1 的包恰好只有一个：这一帧的最后一个包。
                 */
                rtp_fill_header(rtp_packet, marker, ctx->seq, ts, ctx->ssrc);
            }
            else
            {
                rtp_fill_header(rtp_packet, 0, ctx->seq, ts, ctx->ssrc);
            }

            /* 头后面先放两个 FU 字节，再放这一片数据 */
            rtp_packet[RTP_HEADER_SIZE]     = fu_indicator;
            rtp_packet[RTP_HEADER_SIZE + 1] = fu_header;
            memcpy(rtp_packet + RTP_HEADER_SIZE + 2, p, frag);

            /* 包总长 = 12 字节头 + 2 字节 FU + 这一片数据 */
            ret = ctx->send(rtp_packet, RTP_HEADER_SIZE + 2 + frag, ctx->arg);
            if (ret != 0) {
                return ret;   /* 发送失败就停，后面的片发了也拼不成完整 NAL */
            }
            /* 每发一个包序号加 1，分片也一样 */
            ctx->seq++;
            /* 指针后移、剩余减少，准备切下一片 */
            p += frag;
            remain -= frag;
            first = 0;
        }
        return 0;

    }

    return 0;

}
