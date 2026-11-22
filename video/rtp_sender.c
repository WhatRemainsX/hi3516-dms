/*
 * 【说明】本文件的结构（自下而上读更容易）：
 *    rtp_udp_send        最底层：把一个 RTP 包用 sendto 发出去（被 rtp_h264 回调）
 *    rtp_send_frame      一帧 → 拆成多个 NAL → 逐个交给 rtp_h264_send_nal
 *    rtp_on_frame        media_source 的回调：每帧调用一次 rtp_send_frame
 *    rtp_sender_start    创建 socket、初始化打包上下文、订阅码流
 *    rtp_sender_stop     取消订阅、关闭 socket、释放内存
 *
 *   【网络编程小抄】
 *    socket(AF_INET, SOCK_DGRAM, 0)  创建 UDP socket（AF_INET=IPv4，SOCK_DGRAM=数据报/UDP，
 *                                    TCP 则是 SOCK_STREAM）
 *    sendto(fd, 数据, 长度, 标志, 目标地址, 地址长度)  UDP 不需要"连接"，每次发送时指定目标
 *    htons(x)  host to network short：把 16 位整数从主机字节序转成网络字节序（大端）。端口号必须用它
 *    inet_pton("192.168.1.118", ...)  把点分十进制字符串转成 4 字节二进制 IP（已是网络字节序）
 *    UDP 的特点：不保证到达、不保证顺序、没有重传，所以丢包时画面会花；
 *               好处是延迟低，适合实时视频。
 */

#include "rtp_sender.h"
#include "rtp_h264.h"
#include "nal_parser.h"

#include <stdio.h>

#include "media_source.h"     /* 新增：要订阅码流 */

#include <stdlib.h>           /* calloc / free */
#include <string.h>
#include <unistd.h>           /* close / getpid */
#include <time.h>             /* time */
#include <arpa/inet.h>        /* inet_pton / htons */
#include <sys/socket.h>       /* socket / sendto */
#include <netinet/in.h>       /* struct sockaddr_in */

/* 发送一整帧。
 * data/len：media_source 给的一帧码流，里面是若干个"起始码 + NAL"。
 * pts_us  ：这一帧的显示时间，单位微秒。*/
/*
 * 【说明】★一帧的 RTP 打包流程★
 *    1. nal_parse_frame：把这一帧按起始码拆成若干 NAL（例如 [SPS][PPS][IDR]，或只有 [P]）
 *    2. 计算 RTP 时间戳：ts = pts_us * 9 / 100
 *         原因：RTP 视频时钟是 90000 Hz（每秒 90000 个刻度），pts 是微秒（每秒 1000000）。
 *         90000 / 1000000 = 9 / 100。例：pts=33333us（第二帧）→ ts=2999（约 3000 个刻度）。
 *    3. 对每个 NAL 调 rtp_h264_send_nal；只有最后一个 NAL 传 marker=1。
 *   ⚠ 目前一帧的所有包是"瞬间连发"的（没有间隔）。I 帧较大时，会有几十个包在一瞬间发出，
 *     WiFi 环境下容易造成丢包（丢包就会花屏）。以后要优化可以考虑分散发送（pacing），先了解这点。
 */
int rtp_send_frame(rtp_h264_ctx *ctx, const td_u8 *data, td_u32 len, td_u64 pts_us)
{
    nal_frame_info info;/* 存放拆出来的每个 NAL 的位置、长度、类型 */
    uint32_t ts;         /* RTP 时间戳，单位 90kHz */
    td_u32 i;
    int ret;
    /* 参数检查 */
    if (ctx == NULL || data == NULL || len == 0) {
        return -1;
    }

    /*
     * 1. 拆 NAL：找出这一帧里有几个 NAL、各在哪。
     *    info.nals[i].offset 已经跳过起始码，指向 NAL 头；
     *    info.nals[i].len 包含 NAL 头，不含起始码。
     */
    if (nal_parse_frame(data, len, &info) != TD_SUCCESS) {
        printf("rtp: parse frame failed\n");
        return -1;
    }

    /*
     * 2. 微秒 -> 90kHz：ts = pts × 90000 / 1000000 = pts × 9 / 100。
     *    pts_us 是 64 位，乘法不会溢出；结果转成 32 位，超过 2^32 会自然回绕，
     *    RTP 时间戳本来就允许回绕，接收端能正确处理。
     *    整帧只算一次，保证帧内所有包的时间戳相同。
     */
    /*
     * 【说明】(uint32_t) 强制类型转换：把 64 位结果截成 32 位，超过 4294967295 自动回绕。
     *   这是允许的，RTP 协议规定时间戳本来就会回绕，接收端按差值计算即可。
     */
    ts = (uint32_t)(pts_us * 9 / 100);

    /* 3. 逐个发送 NAL，只有最后一个 NAL 传 marker = 1 */
    for(i=0;i<info.count;i++)
    {
        int is_last = 0;
        if(i==info.count-1)
        {
            /* 最后一个 NAL 才传 marker = 1 */
            is_last = 1;
        }
        else
        {
            is_last = 0;
        }

        ret=rtp_h264_send_nal(  ctx,
                            data + info.nals[i].offset,  /* NAL 起点 */
                            info.nals[i].len,            /* NAL 长度 */
                            ts,                          /* 同一帧同一个时间戳 */
                            is_last);                    /* 帧尾才置 M */
        if(ret!=0)
        {
            printf("rtp: send nal %u failed\n", i);
            return ret;
        }
    }

    return 0;
}

/* 一个 RTP 发送器需要的全部东西 */
/*
 * 【说明】这是 rtp_sender.h 里声明的"不透明类型"的真正定义。每个观看者一份：
 *    sock        ：这个观看者自己的 UDP socket
 *    dst         ：目标地址（IP + RTP 端口）；sockaddr_in 是 IPv4 地址结构体
 *    rtp         ：RTP 打包上下文（里面有这个观看者独立的 seq）
 *    sub         ：media_source 订阅句柄
 *    send_errors ：发送失败次数，仅用来控制打印频率（每 100 次才打印一次，避免刷屏）
 */
struct rtp_sender {
    int                sock;         /* UDP socket 的文件描述符 */
    struct sockaddr_in dst;          /* 接收方地址：电脑的 IP + 端口 */
    rtp_h264_ctx       rtp;          /* 打包上下文：序号、SSRC、发包回调 */
    void              *sub;          /* media_source 订阅句柄，取消订阅时要用 */
    unsigned long      send_errors;  /* 发送失败的帧数，用来控制打印频率 */
};

/*
 * 发包回调：rtp_h264_send_nal 每打好一个包就调用一次。
 * 和测试里的 print_head 签名完全一样，只是把"打印"换成了"发到网络"。
 */

 /*
  * 【说明】发包回调：rtp_h264 每组装好一个 RTP 包就调用它一次。
  *   sendto 返回实际发出的字节数，-1 表示失败。UDP 的"发送成功"只代表交给了本机网络协议栈，
  *   不代表对方收到。
  *   ⚠ 编译提示：这里声明了变量 ssize_t ret 但没有使用，gcc 的 -Wall 可能提示 unused variable，
  *     不影响运行，可以删除那一行。
  */
 static int rtp_udp_send(const uint8_t *pkt, size_t len, void *arg)
{
    rtp_sender *s = (rtp_sender *)arg;
    ssize_t n = 0;

    n=sendto(s->sock, pkt, len, 0, (struct sockaddr *)&s->dst, sizeof(s->dst));

    if(n==-1)
    {
        s->send_errors++;
        if((s->send_errors%100)==0)
        {
            printf("rtp: sendto failed %lu times\n", s->send_errors);
        }
        return -1;
    }
    return 0;
}

/*
 * media_source 的回调：每来一帧调用一次。
 * 它运行在 media_source 为这个订阅者创建的线程里，不是编码器取流线程，
 * 所以这里慢一点不会拖慢取流；但如果长期跟不上，队列满了会丢最老的帧。
 */
/*
 * 【说明】这是传给 media_source_subscribe 的回调函数，形状必须是 media_frame_cb：
 *       td_s32 f(const media_frame *frame, void *arg)
 *   arg 就是订阅时传入的 sender 指针，这里强转回来使用。
 *   它运行在 media_source 为这个订阅者创建的专属线程里（不是编码线程，也不是 RTSP 线程）。
 */
static td_s32 rtp_on_frame(const media_frame *frame, void *arg)
{
    rtp_sender *s = (rtp_sender *)arg;
    int ret;

    /* 直接把这一帧交给 RTP 发送器 */
    ret = rtp_send_frame(&s->rtp, frame->data, frame->len, frame->pts);
    if (ret != 0) {
        s->send_errors++;
        if ((s->send_errors % 100) == 0) {
            printf("rtp: send_frame failed %lu times\n", s->send_errors);
        }
        return -1;
    }

    return 0;
}

/*
 * 【说明】创建顺序：calloc → socket → 填目标地址 → rtp_h264_init → media_source_subscribe。
 *   ★订阅放在最后★：一旦订阅，回调马上可能被调用，此时 socket、地址、打包上下文必须已经全部就绪。
 *   失败回滚：订阅失败 → close_sock（关 socket）→ free_socket（释放内存）。
 */
rtp_sender *rtp_sender_start(const char *dst_ip, unsigned short dst_port)
{

   
    rtp_sender *sender = NULL;
    /* 检查参数 */
    if (dst_ip == NULL) {
        return NULL;
    }
    /*  1. 分配发送器，calloc 会把所有字段清零 */
    sender = (rtp_sender *)calloc(1, sizeof(rtp_sender));
    if (sender==NULL) {
        printf("rtp: sender malloc failed\n");
        return NULL;
    }

    /*
     * 2. 创建 UDP socket。
     *    AF_INET    = IPv4
     *    SOCK_DGRAM = 数据报，也就是 UDP（TCP 是 SOCK_STREAM）
     */
    sender->sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sender->sock < 0) {
        printf("rtp: socket failed\n");
        goto free_socket;
    }

    /*
     * 3. 填接收方地址。
     *    端口要用 htons 转成网络字节序，道理和 RTP 头里的 seq 一样。
     *    inet_pton 把字符串 "192.168.1.118" 转成 4 字节的二进制 IP，
     *    它输出的本来就是网络字节序，不用再转。
     */
    sender->dst.sin_family = AF_INET;
    sender->dst.sin_port   = htons(dst_port);
    if (inet_pton(AF_INET, dst_ip, &sender->dst.sin_addr) != 1) {
        printf("rtp: invalid ip %s\n", dst_ip);
        goto close_sock;
    }

    /*初始化打包上下文*/
    /*
     * 【说明】SSRC 用进程号 getpid()：同一台摄像机的所有 RTSP 客户端使用同一个 SSRC。
     *   因为每个客户端的 RTP 都是发往各自不同的 IP:端口，彼此独立，不会混淆，所以共用没问题。
     *   把 rtp_udp_send 当作发包回调传进去，sender 作为回调的私有参数 arg。
     */
    rtp_h264_init(&sender->rtp, (uint32_t)getpid(), rtp_udp_send, sender);
    
    /*最后订阅*/
    sender->sub = media_source_subscribe(rtp_on_frame, sender);
    if (sender->sub == NULL) {
        printf("rtp: subscribe failed\n");
        goto close_sock;
    }
    printf("rtp: sending H.264 to %s:%u\n", dst_ip, (unsigned)dst_port);
    return sender;

close_sock:
    close(sender->sock);
free_socket:
    free(sender);
    return NULL;
}

/*
 * 【说明】停止顺序与启动相反：取消订阅 → 关 socket → free。
 *   ★原因★：media_source_unsubscribe 会等订阅者线程完全退出（pthread_join），
 *   返回之后就保证不会再有人调用 rtp_on_frame，这时关闭 socket 才是安全的。
 *   如果先关了 socket，订阅线程还在 sendto，就会对着一个失效（甚至已被复用）的 fd 发数据。
 */
void rtp_sender_stop(rtp_sender *s)
{
    if (s == NULL) {
        return;
    }

    /*
     * 顺序和 start 正好相反：先取消订阅，再关 socket。
     * media_source_unsubscribe 内部会等订阅者线程退出，
     * 它返回后就保证不会再有回调使用 socket 了，这时关闭才安全。
     */
    media_source_unsubscribe(s->sub);
    close(s->sock);
    free(s);
}
