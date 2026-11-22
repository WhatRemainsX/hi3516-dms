/*
 * 【说明】rtp_sender：把 media_source 送来的每一帧，打成 RTP 包，通过 UDP 发给一个指定的接收端。
 *   一个 rtp_sender = 一个观看者（一个 RTSP 客户端的 PLAY 对应一个）。它做三件事：
 *      ① 创建一个 UDP socket 和目标地址（客户端 IP + 客户端 RTP 端口）
 *      ② 向 media_source 订阅，之后每一帧自动到达 rtp_on_frame 回调
 *      ③ 回调里：拆 NAL（nal_parser）→ 打 RTP 包（rtp_h264）→ sendto 发到网络
 *
 *   数据流：  media_source ─► rtp_on_frame ─► rtp_send_frame ─► rtp_h264_send_nal ─► rtp_udp_send(sendto) ─► 网络
 */

#ifndef RTP_SENDER_H
#define RTP_SENDER_H

#include "ot_type.h"
#include "rtp_h264.h"

/* 把一整帧 H.264（带起始码）打包成 RTP 包发出去 */
int rtp_send_frame(rtp_h264_ctx *ctx, const td_u8 *data, td_u32 len, td_u64 pts_us);

/* 第 5 步：RTP 发送器。具体内容藏在 .c 里，外面只拿一个指针用 */
/*
 * 【说明】"不透明类型"：头文件里只声明有这么个结构体，不给出里面的字段（具体定义藏在 .c 里）。
 *   外部代码只能拿到指针，不能直接读写它的字段，只能通过 start/stop 函数操作，避免被乱改。
 *   rtsp_server.h 里的 rtsp_server 也是同样的写法。
 */
typedef struct rtp_sender rtp_sender;

/* 创建 UDP socket 并订阅码流，之后每一帧都会自动发往 dst_ip:dst_port */
/*
 * 【说明】start = 创建 + 订阅。返回的指针就是这个观看者的"句柄"，stop 时要原样传回来。
 *   RTSP 服务器在客户端 PLAY 时调用它；TEARDOWN、断线或服务器退出时调用 rtp_sender_stop。
 */
rtp_sender *rtp_sender_start(const char *dst_ip, unsigned short dst_port);

/* 取消订阅、关闭 socket、释放内存 */
void rtp_sender_stop(rtp_sender *s);

#endif
