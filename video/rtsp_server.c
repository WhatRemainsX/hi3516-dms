/*
 * @file rtsp_server.c
 * @brief 最简 RTSP 服务器：多客户端，RTP over UDP。
 *
 * 结构：
 *   rtsp_server_start  创建监听 socket（总机），启动 RTSP 线程
 *   rtsp_server_thread 循环等待新连接，accept 得到专线 client_fd
 *   rtsp_handle_client 在专线上收数据，攒成完整请求后逐条解析
 *   rtsp_server_stop   通知线程退出，回收资源
 *
 * 当前能力：同时服务多个客户端（最多 RTSP_MAX_CLIENTS 个，每个客户端一个线程）；
 * 支持 OPTIONS / DESCRIBE / SETUP / PLAY / TEARDOWN，视频通过 UDP 发送 RTP。
 * 暂不支持：TCP interleaved 传输（SETUP 会回复 461）、音频、鉴权。
 */
/*
 * 【说明】━━━━━━ 先看全局地图（RTSP 服务器怎么运转） ━━━━━━
 *
 *   RTSP 是"遥控器"：VLC 通过 TCP 连接，用文本命令控制摄像机；真正的视频走另一条 UDP 通道（RTP）。
 *   典型对话：
 *        VLC                                         摄像机
 *         │──── OPTIONS  ───────────────────────────►│  问：你支持哪些命令？
 *         │◄─── 200 OK  Public: OPTIONS,DESCRIBE,... │
 *         │──── DESCRIBE ───────────────────────────►│  问：你的流是什么？
 *         │◄─── 200 OK  + SDP（含 SPS/PPS）           │
 *         │──── SETUP  client_port=56042-56043 ─────►│  说：我在这两个 UDP 端口收视频
 *         │◄─── 200 OK  Session: 1A2B3C4D             │  答：会话号给你
 *         │──── PLAY   Session: 1A2B3C4D ───────────►│  说：开始发吧
 *         │◄─── 200 OK                                │  → 此时创建 rtp_sender，视频开始走 UDP
 *         │◄════ RTP/UDP 视频包 ═════════════════════│
 *         │──── TEARDOWN ───────────────────────────►│  说：结束
 *         │◄─── 200 OK                                │  → 销毁 rtp_sender
 *
 *   【线程结构】
 *      主线程 main
 *        └─ rtsp_server_thread      只负责 accept（接电话），每来一个客户端就生成一个专线线程
 *             ├─ rtsp_client_thread (客户端1) ─► rtsp_handle_client：收请求→解析→回复，直到断开
 *             ├─ rtsp_client_thread (客户端2) ─► rtsp_handle_client
 *             └─ ……
 *      每个客户端 PLAY 之后，还会有一个 rtp_sender（它订阅 media_source，有自己的线程）。
 *
 *   【文件阅读顺序】本文件的函数是"先小零件，后大流程"排列的（C 要求先定义后使用）：
 *      rtsp_session_stop_rtp / rtsp_send_all / rtsp_reply     发送相关的小工具
 *      rtsp_build_sdp                                         生成 SDP 描述
 *      rtsp_on_setup / on_play / on_teardown / handle_request 各命令的处理
 *      wait_readable / rtsp_parse_header_line / rtsp_parse_request   接收与解析
 *      rtsp_handle_client                                     一个客户端的完整对话循环
 *      rtsp_client_thread / rtsp_server_thread                线程入口
 *      rtsp_server_start / rtsp_server_stop                   对外接口（启动/停止）
 *   建议先看最后的 start/stop，再看 server_thread → client_thread → handle_client，最后看各零件。
 */
#include "rtsp_server.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>        /* strncasecmp：忽略大小写比较字符串 */
#include <errno.h>          /* errno、EINTR */
#include <unistd.h>         /* close */
#include <pthread.h>
#include <stdatomic.h>      /* atomic_int、atomic_load、atomic_store */
#include <arpa/inet.h>      /* inet_ntop、htons、ntohs、htonl */
#include <netinet/in.h>     /* struct sockaddr_in、INADDR_ANY */
#include <sys/socket.h>     /* socket、bind、listen、accept、recv */
#include <sys/select.h>     /* select、fd_set */
#include "nal_parser.h"     /* DESCRIBE 要用 nal_get_sps_pps 和 nal_base64_encode */
#include <time.h>           /* time：生成会话 ID 用 */
#include "rtp_sender.h"     /* PLAY 时启动 RTP，TEARDOWN 时停止 */

/* 服务器上下文：外部只拿到指针，看不到内部（不透明类型） */
struct rtsp_server
{
    int listen_fd;          /* 监听套接字（总机），只用来 accept，不收发数据 */
    pthread_t thread;       /* RTSP 线程句柄 */
    /*
     * 【说明】atomic_int（原子整数）：多个线程同时读写的变量，用 atomic_xxx 函数操作，
     *   保证读写不会被"撕裂"，也不会被编译器优化掉：
     *      atomic_load(&x)          读取
     *      atomic_store(&x, v)      写入
     *      atomic_fetch_add(&x, 1)  原子地 +1（返回旧值）；atomic_fetch_sub 同理 -1
     *   这里 running 由主线程写（stop 时置 0），accept 线程和客户端线程读；
     *   active_clients 由 accept 线程 +1、客户端线程 -1，主线程等它降到 0。
     */
    atomic_int running;     /* 线程运行标志：主线程写、RTSP 线程读，所以用原子变量 */
    atomic_int active_clients;  /* 当前有多少个客户端连接，主线程和 RTSP 线程都可能读写 */
};

/*
 * 每个客户端的接收缓冲区大小。
 * 一条 RTSP 请求通常只有两三百字节，4KB 足够容纳一条完整请求，
 * 外加一点粘在后面的下一条请求。
 */
/*
 * 【说明】三个限制值。栈大小 256 KB：每个客户端线程用到的局部数组（4 KB 的 buf 等）很小，
 *   默认每线程栈 8 MB 是虚拟地址空间，线程多了会浪费，所以主动调小。
 */
#define RTSP_REQ_BUF_SIZE 4096
/* 同时最多服务多少个客户端；再多的连接直接断开 */
#define RTSP_MAX_CLIENTS 8

/* 客户端线程的栈大小：256KB 足够，比默认的 8MB 省得多 */
#define RTSP_CLIENT_STACK_SIZE (256 * 1024)

/* 从一条请求中解析出来的字段 */
typedef struct {
    char method[16];        /* 方法：OPTIONS / DESCRIBE / SETUP / PLAY / TEARDOWN */
    char url[256];          /* 请求地址，例如 rtsp://192.168.1.222:554/live.sdp */
    int  cseq;              /* 请求序号，回复时必须原样带回；-1 表示请求里没有 CSeq */
    int  client_rtp_port;   /* SETUP 的 client_port 前一个：客户端接收 RTP 的 UDP 端口 */
    int  client_rtcp_port;  /* SETUP 的 client_port 后一个：客户端接收 RTCP 的 UDP 端口 */
    char session[64];       /* 会话 ID，SETUP 之后的 PLAY / TEARDOWN 会带上 */
} rtsp_request;

/* 一个客户端连接的会话状态 */
typedef enum {
    RTSP_STATE_INIT,        /* 刚连上，或 TEARDOWN 之后 */
    RTSP_STATE_READY,       /* SETUP 完成，知道往哪发了，还没开始发 */
    RTSP_STATE_PLAYING      /* PLAY 之后，正在发 RTP */
} rtsp_state;

/*
 * 每个连接一个，整个连接期间一直存在。
 * rtsp_request 记录"这一句话说了什么"，用完就丢；
 * rtsp_session 记录"这个客户端的整体状态"，在多条请求之间保留。
 */
typedef struct {
    int        fd;                              /* 和客户端通信的 TCP 连接 */
    char       client_ip[INET_ADDRSTRLEN];      /* 客户端 IP，来自 accept */
    int        client_rtp_port;                 /* 来自 SETUP */
    int        client_rtcp_port;                /* 来自 SETUP */
    char       session_id[32];                  /* SETUP 时生成 */
    rtsp_state state;
    rtp_sender *rtp;                            /* PLAY 时创建，TEARDOWN 或断开时销毁 */
    /*
     * 【说明】seed：随机数种子。rand_r(&seed) 是可重入（线程安全）的随机数函数：
     *   每个会话用自己的 seed，多个线程同时调用也不会互相干扰（普通的 rand() 共用一个全局状态，不安全）。
     */
    unsigned int seed;                          /* 生成会话 ID 的随机种子 */
} rtsp_session;

/* 停止这个会话的 RTP 发送（没在发就什么也不做） */
/*
 * 【说明】"置空防重复"是常用的防御写法：stop 完把指针设成 NULL，以后再次调用就什么都不做，
 *   所以 TEARDOWN、断线、服务器退出三条路径都可以放心调用它。
 */
static void rtsp_session_stop_rtp(rtsp_session *sess)
{
    if (sess->rtp != NULL) {
        rtp_sender_stop(sess->rtp);     /* 取消订阅、关 socket、释放 */
        sess->rtp = NULL;               /* 置空，防止重复停止 */
    }
}

/*
 * 把 len 字节全部发出去。
 * send 和 recv 一样，一次不一定能发完：内核发送缓冲区快满时，
 * 可能只发出一部分，返回实际发送的字节数，所以要循环直到发完。
 */
/*
 * 【说明】send / recv 的一个常见坑：TCP 是字节流，send 不保证一次发完，返回实际发出的字节数。
 *   所以要循环：已发的跳过(data += n)，剩下的继续发(len -= n)，直到发完。
 *   EINTR：send 被信号打断（不是真正错误），重试即可。errno 是系统保存"最近一次错误码"的全局变量，
 *   strerror(errno) 把它转成文字。
 */
static int rtsp_send_all(int fd, const char *data, size_t len)
{
    while (len > 0) {
        /*
         * MSG_NOSIGNAL：对方已经断开时，send 返回错误，而不是向进程发送 SIGPIPE 信号。
         * 不加的话，SIGPIPE 的默认动作是直接杀死整个程序。
         */
        ssize_t n = send(fd, data, len, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) {
                continue;               /* 被信号打断，重试 */
            }
            printf("rtsp: send failed: %s\n", strerror(errno));
            return -1;
        }
        data += n;                      /* 已发的跳过 */
        len  -= (size_t)n;              /* 还剩多少没发 */
    }
    return 0;
}

/*
 * 发送一条回复，格式：
 *   RTSP/1.0 <status>\r\n
 *   CSeq: <cseq>\r\n          ← 必须原样带回请求的 CSeq
 *   Server: my_camera\r\n
 *   <headers>                 ← 额外头部，每行自己带 \r\n
 *   Content-Length: <n>\r\n   ← 正文的字节数，没有正文就是 0
 *   \r\n                      ← 空行，头部结束
 *   <body>                    ← 正文
 *
 * status ：例如 "200 OK"
 * headers：额外头部，没有就传 NULL
 * body   ：正文，没有就传 NULL
 */
/*
 * 【说明】所有回复都经过这个函数。注意 RTSP 回复和 HTTP 很像：状态行 + 头部 + 空行 + 正文。
 *   snprintf 的好处：永远不会写超过 sizeof(out)（比 sprintf 安全），返回"本应写入的长度"，
 *   用它判断是否被截断。头部/正文参数传 NULL 时，用 "" 代替，避免 %s 遇到 NULL 崩溃。
 */
static int rtsp_reply(int fd, int cseq, const char *status,
                      const char *headers, const char *body)
{
    char out[4096];
    size_t body_len = (body != NULL) ? strlen(body) : 0;
    int n;

    /*
     * snprintf：和 printf 一样按格式拼字符串，但结果写进 out，最多写 sizeof(out) 个字节。
     * 返回值是"本应写入的总长度"，如果 >= sizeof(out)，说明被截断了。
     */
    n = snprintf(out, sizeof(out),
                 "RTSP/1.0 %s\r\n"
                 "CSeq: %d\r\n"
                 "Server: my_camera\r\n"
                 "%s"
                 "Content-Length: %zu\r\n"
                 "\r\n"
                 "%s",
                 status,
                 cseq,
                 (headers != NULL) ? headers : "",
                 body_len,                       /* 用 strlen 算出的精确长度 */
                 (body != NULL) ? body : "");
    if (n < 0 || (size_t)n >= sizeof(out)) {
        printf("rtsp: reply too large\n");
        return -1;
    }

    printf("rtsp: >>> reply %s (cseq=%d)\n", status, cseq);
    return rtsp_send_all(fd, out, (size_t)n);
}

/*
 * 生成 SDP，写进 sdp 缓冲区。成功返回 0。
 * 这就是你以前手写的 test.sdp，现在由程序自动生成。
 */
/*
 * 【说明】SDP（Session Description Protocol，会话描述协议）：一段纯文本，告诉客户端"这个流有什么、怎么解码"。
 *   关键行：
 *     m=video 0 RTP/AVP 96        媒体行：视频；RTP over UDP；载荷类型 96
 *     a=rtpmap:96 H264/90000      96 号载荷 = H.264，时钟 90kHz
 *     a=fmtp:96 packetization-mode=1;sprop-parameter-sets=<SPS>,<PPS>
 *                                 packetization-mode=1：允许 FU-A 分片；sprop-...：Base64 的 SPS/PPS，
 *                                 客户端不用等关键帧就能先初始化解码器
 *     a=control:...               后续 SETUP 用的地址
 *   刚开机还没出过 IDR 帧时 SPS/PPS 为空，就不写 sprop，让播放器从码流里自己取。
 */
static int rtsp_build_sdp(const char *url, char *sdp, size_t size)
{
    td_u8 sps[NAL_SPS_MAX_SIZE];
    td_u8 pps[NAL_PPS_MAX_SIZE];
    td_u32 sps_len = 0;
    td_u32 pps_len = 0;
    char sps_b64[512];      /* 256 字节 Base64 编码后约 344 个字符 */
    char pps_b64[128];      /* 64 字节编码后约 88 个字符 */
    char fmtp[800];
    int n;

    /*
     * 从 nal_parser 的缓存里取 SPS/PPS，再转成 Base64。
     * 编码器刚启动、还没出过关键帧时取不到，这时就不写 sprop-parameter-sets，
     * VLC 会等码流里的 SPS/PPS（每个 IDR 前都有）。
     */
    if (nal_get_sps_pps(sps, &sps_len, pps, &pps_len) == TD_SUCCESS &&
        nal_base64_encode(sps, sps_len, sps_b64, sizeof(sps_b64)) == TD_SUCCESS &&
        nal_base64_encode(pps, pps_len, pps_b64, sizeof(pps_b64)) == TD_SUCCESS) {
        snprintf(fmtp, sizeof(fmtp),
                 "a=fmtp:96 packetization-mode=1;sprop-parameter-sets=%s,%s\r\n",
                 sps_b64, pps_b64);
    } else {
        snprintf(fmtp, sizeof(fmtp), "a=fmtp:96 packetization-mode=1\r\n");
    }

    /*
     * SDP 每行也用 \r\n 结尾。
     * m=video 0、c=0.0.0.0：端口和地址都还没定，等 SETUP 时再商量。
     * 第一个 a=control：整个会话的控制地址，PLAY 时用；
     * 最后一个 a=control：这条视频轨道的地址，VLC 下一步 SETUP 就用它。
     */
    n = snprintf(sdp, size,
                 "v=0\r\n"
                 "o=- 0 0 IN IP4 0.0.0.0\r\n"
                 "s=my_camera\r\n"
                 "t=0 0\r\n"
                 "a=control:%s\r\n"
                 "a=range:npt=0-\r\n"
                 "m=video 0 RTP/AVP 96\r\n"
                 "c=IN IP4 0.0.0.0\r\n"
                 "a=rtpmap:96 H264/90000\r\n"
                 "%s"
                 "a=control:%s/track0\r\n",
                 url, fmtp, url);
    if (n < 0 || (size_t)n >= size) {
        return -1;
    }
    return 0;
}

/* SETUP：记下客户端的 RTP 端口，生成会话 ID */
/*
 * 【说明】SETUP：客户端告诉我们"我在哪个 UDP 端口收视频"（client_port=偶数-奇数：偶数给 RTP，奇数给 RTCP）。
 *   我们做两件事：记住端口、生成会话 ID（Session），状态 INIT → READY。
 *   此时还没有开始发送视频，要等 PLAY。
 *   rand_r(&sess->seed) 生成随机数，"%08X" 打印成 8 位大写十六进制（如 1A2B3C4D）。
 */
static void rtsp_on_setup(rtsp_session *sess, const rtsp_request *req)
{
    char headers[256];

    /*
     * 第一版只支持 UDP：请求里必须有 client_port。
     * 如果客户端要求 TCP 方式（RTP/AVP/TCP;interleaved=0-1），就没有 client_port，
     * 回复 461，VLC 会退回 UDP 方式重试。
     */
    if (req->client_rtp_port <= 0) {
        rtsp_reply(sess->fd, req->cseq, "461 Unsupported Transport", NULL, NULL);
        return;
    }

    /* 从 req 复制到 sess：req 马上就没了，sess 会一直保留到 PLAY */
    sess->client_rtp_port  = req->client_rtp_port;
    sess->client_rtcp_port = req->client_rtcp_port;

    /* 生成 8 位十六进制的会话 ID；同一个连接重复 SETUP 时沿用原来的 */
    if (sess->session_id[0] == '\0') {
        snprintf(sess->session_id, sizeof(sess->session_id), "%08X", rand_r(&sess->seed));
    }
    sess->state = RTSP_STATE_READY;

    /* 回复时把 Transport 原样确认一遍，并告诉客户端会话 ID */
    snprintf(headers, sizeof(headers),
             "Transport: RTP/AVP;unicast;client_port=%d-%d\r\n"
             "Session: %s\r\n",
             sess->client_rtp_port, sess->client_rtcp_port, sess->session_id);
    rtsp_reply(sess->fd, req->cseq, "200 OK", headers, NULL);

    printf("rtsp: session %s -> RTP to %s:%d\n",
           sess->session_id, sess->client_ip, sess->client_rtp_port);
}

/* PLAY：开始播放（第 5 步在这里启动 RTP） */
/*
 * 【说明】PLAY：真正开始发视频。创建 rtp_sender（向 media_source 订阅，从此每帧自动发出），
 *   状态 READY → PLAYING。重复 PLAY 不会重复创建（检查 sess->rtp == NULL）。
 *   错误码：455 = 当前状态不允许该命令（没 SETUP 就 PLAY）；500 = 服务器内部错误。
 */
static void rtsp_on_play(rtsp_session *sess, const rtsp_request *req)
{
    char headers[128];

    /* 没有 SETUP 就 PLAY，不知道往哪发，拒绝 */
    if (sess->state == RTSP_STATE_INIT) {
        rtsp_reply(sess->fd, req->cseq, "455 Method Not Valid in This State", NULL, NULL);
        return;
    }

    /* 还没在发才启动，重复 PLAY 不会启动第二个发送器 */
    if (sess->rtp == NULL) {
        sess->rtp = rtp_sender_start(sess->client_ip, (unsigned short)sess->client_rtp_port);
        if (sess->rtp == NULL) {
            rtsp_reply(sess->fd, req->cseq, "500 Internal Server Error", NULL, NULL);
            return;
        }
    }
    sess->state = RTSP_STATE_PLAYING;

    snprintf(headers, sizeof(headers),
             "Session: %s\r\n"
             "Range: npt=0.000-\r\n",
             sess->session_id);
    rtsp_reply(sess->fd, req->cseq, "200 OK", headers, NULL);
    printf("rtsp: session %s PLAYING\n", sess->session_id);
}

/* TEARDOWN：结束播放 */
/*
 * 【说明】TEARDOWN：客户端礼貌地告别。停止 RTP 发送（取消订阅、关 socket），状态回到 INIT。
 *   注意：这里只是结束播放，TCP 连接还在；客户端随后通常会自己关闭连接，handle_client 里 recv 返回 0 后退出。
 */
static void rtsp_on_teardown(rtsp_session *sess, const rtsp_request *req)
{
    char headers[64];

    rtsp_session_stop_rtp(sess);
    sess->state = RTSP_STATE_INIT;

    snprintf(headers, sizeof(headers), "Session: %s\r\n", sess->session_id);
    rtsp_reply(sess->fd, req->cseq, "200 OK", headers, NULL);
    printf("rtsp: session %s TEARDOWN\n", sess->session_id);
}

/* 根据请求的方法名，决定怎么回复 */
/*
 * 【说明】命令分发器（像"总机转接"）：先检查 PLAY/TEARDOWN 带的 Session 是否匹配，
 *   再按方法名调用对应处理函数。454 = Session Not Found；501 = 不支持的方法。
 *   strcmp(a, b) == 0 表示两个字符串内容相同。
 */
static void rtsp_handle_request(rtsp_session *sess, const rtsp_request *req)
{
    /*
     * PLAY、TEARDOWN 必须带上 SETUP 时分配的会话 ID，
     * 对不上说明不是这个会话的请求，回复 454。
     */
    if (strcmp(req->method, "PLAY") == 0 || strcmp(req->method, "TEARDOWN") == 0) {
        if (sess->session_id[0] == '\0' || strcmp(req->session, sess->session_id) != 0) {
            rtsp_reply(sess->fd, req->cseq, "454 Session Not Found", NULL, NULL);
            return;
        }
    }

    if (strcmp(req->method, "OPTIONS") == 0) {
        rtsp_reply(sess->fd, req->cseq, "200 OK",
                   "Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN\r\n", NULL);

    } else if (strcmp(req->method, "DESCRIBE") == 0) {
        char sdp[1024];

        if (rtsp_build_sdp(req->url, sdp, sizeof(sdp)) != 0) {
            rtsp_reply(sess->fd, req->cseq, "500 Internal Server Error", NULL, NULL);
            return;
        }
        rtsp_reply(sess->fd, req->cseq, "200 OK",
                   "Content-Type: application/sdp\r\n", sdp);

    } else if (strcmp(req->method, "SETUP") == 0) {
        rtsp_on_setup(sess, req);

    } else if (strcmp(req->method, "PLAY") == 0) {
        rtsp_on_play(sess, req);

    } else if (strcmp(req->method, "TEARDOWN") == 0) {
        rtsp_on_teardown(sess, req);

    } else {
        rtsp_reply(sess->fd, req->cseq, "501 Not Implemented", NULL, NULL);
    }
}

/*
 * 等 fd 可读，最多等 timeout_ms 毫秒。
 * 返回 >0：可读；0：超时；<0：出错。
 *
 * 对监听 fd 来说，"可读"表示有新连接在排队，可以 accept；
 * 对客户端 fd 来说，"可读"表示有数据到达，或者对方关闭了连接，可以 recv。
 *
 * 设置超时的目的：不让线程永远卡在 accept / recv 里，
 * 每隔一段时间回到循环开头检查 running，rtsp_server_stop 才能让线程退出。
 */
/*
 * 【说明】select 的封装：等待 fd 变成"可读"，最多等 timeout_ms。
 *   为什么不直接 accept/recv 一直阻塞？因为阻塞时线程没法响应 "stop"。
 *   现在每秒醒一次，检查 running 标志，rtsp_server_stop 最多 1 秒就能让所有线程退出。
 */
static int wait_readable(int fd, int timeout_ms)
{
    fd_set read_fds;            /* 要监视"可读"的 fd 集合 */
    struct timeval timeout;     /* 超时时间 */
    int ret;

    /* 每次调用都重新设置：select 返回时会改写 read_fds 和 timeout */
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    timeout.tv_sec = timeout_ms / 1000;              /* 秒 */
    timeout.tv_usec = (timeout_ms % 1000) * 1000;    /* 剩余的毫秒换成微秒 */

    /* 第一个参数是"最大 fd + 1"，内核只检查比它小的 fd */
    ret = select(fd + 1, &read_fds, NULL, NULL, &timeout);
    if (ret < 0 && errno == EINTR) {
        /* 被信号打断（例如按 Ctrl+C）不是真正的错误，当作超时，让调用者继续循环 */
        return 0;
    }

    return ret;
}

/*
 * 解析一行头部，例如 "CSeq: 3"。
 * 认识的字段填进 req，不认识的（User-Agent、Accept、Range 等）直接忽略。
 * 头部名字不区分大小写，所以用 strncasecmp 而不是 strncmp。
 */
/*
 * 【说明】本函数用到的几个字符串函数：
 *    strncasecmp(a, b, n)   忽略大小写比较前 n 个字符（"CSeq" "cseq" "CSEQ" 视为相同）
 *    atoi("  3")             字符串转整数，会自动跳过前导空格
 *    sscanf(str, 格式, 变量地址...)  按格式从字符串里"扫描"出数据（和 scanf 类似，但来源是字符串）
 *         "%d-%d" 读两个用 - 连接的整数；"%63[^;\r\n]" 读最多 63 个"不是分号/回车/换行"的字符
 *    strstr(大串, 小串)      在大串里找小串，找到返回位置指针，找不到返回 NULL
 */
static void rtsp_parse_header_line(const char *line, rtsp_request *req)
{
    //如果以CSeq:开头，则解析CSeq字段
    if (strncasecmp(line, "CSeq:", 5) == 0) {
        /* line + 5 跳过 "CSeq:" 这 5 个字符；atoi 会自动跳过前面的空格 */
        req->cseq = atoi(line + 5);

    } else if (strncasecmp(line, "Session:", 8) == 0) {
        const char *p = line + 8;       /* 跳过 "Session:" 这 8 个字符 */

        while (*p == ' ') {             /* 再跳过冒号后面的空格 */
            p++;
        }
        /*
         * %63[^;\r\n]：读取除 ';'、'\r'、'\n' 以外的字符，最多 63 个。
         * Session 后面可能跟着 ";timeout=60"，这里只取分号前面的会话 ID；
         * 限制 63 个是为了给 session[64] 留出结尾的 '\0'，防止越界。
         */
        sscanf(p, "%63[^;\r\n]", req->session);

    } else if (strncasecmp(line, "Transport:", 10) == 0) {
        /*
         * Transport 行很长，例如：
         *   Transport: RTP/AVP;unicast;client_port=56042-56043
         * 用 strstr 在整行里找 "client_port="，返回它的位置。
         */
        const char *p = strstr(line, "client_port=");

        if (p != NULL) {
            /* "client_port=" 有 12 个字符，p + 12 指向第一个数字 */
            sscanf(p + 12, "%d-%d", &req->client_rtp_port, &req->client_rtcp_port);
        }
    }
}

/*
 * 解析一条完整请求（以 \r\n\r\n 结尾）。
 * 成功返回 0，格式不对返回 -1。
 *
 * 切行技巧：找到每行末尾的 "\r\n"，把 '\r' 改成 '\0'，
 * 这样不用复制，每一行就成了一个独立的字符串。
 * 代价是会修改传进来的缓冲区，所以调用者要在解析之前算好请求长度。
 */
/*
 * 【说明】解析一整条请求的"切行技巧"：不复制字符串，而是在每行末尾的 \r 处写入 '\0'
 *   （C 字符串以 '\0' 结尾），于是每一行自然变成独立的字符串，直接传给 rtsp_parse_header_line。
 *   memset(req, 0, sizeof(*req))：先整体清零，再逐项填写。
 */
static int rtsp_parse_request(char *text, rtsp_request *req)
{
    char *line;     /* 当前行的开头 */
    char *next;     /* 当前行末尾 "\r\n" 的位置 */

    memset(req, 0, sizeof(*req));
    req->cseq = -1;                     /* 先假设没有 CSeq，找到再覆盖 */

    /* 1. 第一行：方法 URL 版本，例如 "DESCRIBE rtsp://... RTSP/1.0" */
    line = text;
    next = strstr(line, "\r\n");
    if (next == NULL) {
        return -1;
    }
    *next = '\0';                       /* 把第一行截成独立字符串 */

    /*
     * %15s、%255s：读取一段不含空格的字符串，并限制长度，
     * 防止对方发来超长内容写出 method[16]、url[256] 的边界。
     * 版本号 "RTSP/1.0" 暂时不需要，不读取。
     */
    if (sscanf(line, "%15s %255s", req->method, req->url) != 2) {
        return -1;
    }

    /* 2. 后面每一行都是头部，直到遇到空行为止 */
    line = next + 2;                    /* 跳过 "\r\n"，指向下一行开头 */
    while ((next = strstr(line, "\r\n")) != NULL) {
        *next = '\0';                   /* 截断当前行 */
        if (line[0] == '\0') {          /* 空行：头部结束 */
            break;
        }
        rtsp_parse_header_line(line, req);
        line = next + 2;                /* 移到下一行 */
    }
    return 0;
}

/*
 * 和一个已连接的客户端对话，直到它断开或服务器要停止。
 *
 * TCP 是字节流，一次 recv 不一定正好是一条请求：
 *   - 可能两条粘在一起（粘包）
 *   - 可能一条被拆成两次收到（拆包）
 * 所以把收到的数据都追加到 buf 里，再按 "\r\n\r\n" 切出完整请求逐条处理，
 * 不完整的部分留在 buf 里，等下一次 recv 补齐。
 */
/*
 * 【说明】★一个客户端的完整生命周期★（运行在这个客户端自己的线程里）。
 *   循环：等待可读(1秒超时) → recv 追加到 buf → 按 \r\n\r\n 切出完整请求 → 解析 → 处理 → 把已处理的部分移走。
 *   退出循环的情况：对方断开(recv 返回 0)、出错、缓冲区满、服务器要停止。
 *   退出后做收尾：如果还在播放，停止 RTP；关闭这条 TCP 连接。
 *   recv(fd, buf, 最大字节, 0)：返回收到的字节数；0 表示对方已正常关闭；-1 表示错误。
 */
static void rtsp_handle_client(rtsp_server *server, int client_fd, const struct sockaddr_in *addr)
{
    char buf[RTSP_REQ_BUF_SIZE];    /* 攒请求的缓冲区 */
    size_t used = 0;                /* buf 里已经有多少字节 */
    char ip[INET_ADDRSTRLEN];       /* 客户端 IP 的字符串形式 */
    rtsp_session sess;
    
    /* inet_ntop：把二进制 IP 转成 "192.168.1.118" 这样的字符串，是 inet_pton 的反向操作 */
    inet_ntop(AF_INET, &addr->sin_addr, ip, sizeof(ip));

    memset(&sess, 0, sizeof(sess));
    /*
     * 【说明】⚠ 小问题：种子只用了当前时间（秒）。如果两个客户端在同一秒内连上，它们的种子相同，
     *   rand_r 会生成相同的会话 ID（Session），两个客户端的 PLAY/TEARDOWN 就会混淆。
     *   建议把客户端地址混进去，让每个连接的种子都不同：
     *       sess.seed = (unsigned)time(NULL)
     *                 ^ ((unsigned)ntohs(addr->sin_port) << 16)
     *                 ^ ntohl(addr->sin_addr.s_addr);
     *   （同一秒内不同客户端的端口号一定不同，因此种子必然不同。）
     */
    sess.seed = (unsigned int)time(NULL);   /* 生成会话 ID 的随机种子 */
    sess.state = RTSP_STATE_INIT;
    sess.fd = client_fd;
    snprintf(sess.client_ip, sizeof(sess.client_ip), "%s", ip);

    /* sin_port 是客户端这条 TCP 连接的端口，注意不是 RTP 端口 */
    printf("rtsp: client %s:%u connected\n", ip, (unsigned)ntohs(addr->sin_port));

    while (atomic_load(&server->running)) {
        ssize_t n;
        char *end;
        int ret = wait_readable(client_fd, 1000);   /* 最多等 1 秒 */

        if (ret < 0) {
            printf("rtsp: select failed: %s\n", strerror(errno));
            break;
        }
        if (ret == 0) {
            continue;                   /* 这 1 秒没有数据，回去检查 running */
        }

        /* 缓冲区满了还凑不出一条完整请求，说明对方发的不是正常 RTSP，断开 */
        if (used >= sizeof(buf) - 1) {
            printf("rtsp: request too large, closing\n");
            break;
        }

        /*
         * 追加到已有数据后面：从 buf + used 开始放，
         * 不覆盖上一次没收完的半条请求；最多再收这么多，留 1 字节放 '\0'。
         */
        n = recv(client_fd, buf + used, sizeof(buf) - 1 - used, 0);
        if (n < 0) {
            printf("rtsp: recv failed: %s\n", strerror(errno));
            break;
        }
        if (n == 0) {
            /* recv 返回 0：对方关闭了连接（例如 VLC 直接断开而没有发 TEARDOWN） */
            printf("rtsp: client %s closed the connection\n", ip);
            break;
        }
        used += (size_t)n;
        buf[used] = '\0';               /* 保持是字符串，strstr 才能使用 */

        /*
         * 缓冲区里有几条完整请求就处理几条，所以用 while 而不是 if：
         * 两条粘在一起时，要处理两次。
         */
        while ((end = strstr(buf, "\r\n\r\n")) != NULL) {
            /*
             * 这条请求的总长度 = 空行之前的部分 + 4 个字节的 "\r\n\r\n"。
             * 必须在解析之前计算：解析会把 '\r' 改成 '\0'，之后就找不到 "\r\n\r\n" 了。
             */
            size_t req_len = (size_t)(end - buf) + 4;
            rtsp_request req;

            if (rtsp_parse_request(buf, &req) == 0) {
                printf("rtsp: [%s] url=%s cseq=%d", req.method, req.url, req.cseq);
                if (req.client_rtp_port > 0) {          /* 只有 SETUP 才带 client_port */
                    printf(" client_port=%d-%d", req.client_rtp_port, req.client_rtcp_port);
                }
                if (req.session[0] != '\0') {           /* PLAY / TEARDOWN 才带 Session */
                    printf(" session=%s", req.session);
                }
                printf("\n");
                rtsp_handle_request(&sess, &req);     /* ← 新增 */
            } else {
                printf("rtsp: bad request\n");
            }

            /*
             * 把处理完的这条从缓冲区里去掉：后面剩下的内容挪到开头。
             * 源区域 buf + req_len 和目的区域 buf 可能重叠，所以用 memmove，
             * memcpy 遇到重叠时结果是不确定的。
             */
            memmove(buf, buf + req_len, used - req_len);
            used -= req_len;
            buf[used] = '\0';
        }
        /* 走到这里，buf 里要么空了，要么是半条请求，留着等下一次 recv */
    }

    /* 连接断开时还在播放，就要停止（第 5 步在这里调用 rtp_sender_stop） */
    if (sess.state == RTSP_STATE_PLAYING) {
        printf("rtsp: session %s ended by disconnect\n", sess.session_id);
    }
    rtsp_session_stop_rtp(&sess);
    close(client_fd);
}

/* 交给客户端线程的全部信息：放堆上，addr 是值拷贝（不是指针） */
typedef struct {
    rtsp_server       *server;
    int                client_fd;
    struct sockaddr_in addr;
} rtsp_client_context;

/*
 * 【说明】客户端线程入口：handle_client 跑完（客户端断开）后，释放 ctx 并把计数 -1。
 *   ctx（context，上下文）是 accept 线程用 malloc 在堆上分配、专门交给这个线程的"包裹"：
 *   里面有 server 指针、client_fd、客户端地址（值拷贝）。不能传 accept 线程的局部变量的地址，
 *   因为 accept 线程的下一轮循环会改写它们。包裹由这个线程负责 free。
 */
static void *rtsp_client_thread(void *arg)
{
    rtsp_client_context *ctx = (rtsp_client_context *)arg;

    /*
     * 【说明】✔ 这里的写法是正确的（已修复 use-after-free）：
     *   free(ctx) 之后就不能再访问 ctx 里的任何东西了，所以要在 free 之前，
     *   先把需要的 server 指针复制到局部变量里；free 之后只使用这个局部变量。
     *   而且 atomic_fetch_sub 必须是线程的最后一件事：这个 -1 一执行，
     *   rtsp_server_stop 可能立刻 free(server)，之后任何访问都不安全。
     */
    rtsp_server *server = ctx->server;          // 先取出来
    rtsp_handle_client(server, ctx->client_fd, &ctx->addr);
    free(ctx);      /* 线程结束前释放上下文 */

    /* 必须是最后一件事：这一行之后不能再访问 server */
    atomic_fetch_sub(&server->active_clients, 1);
    return NULL;
}

/* RTSP 线程：只负责 accept，每接到一个客户端就交给一个新线程 */
/*
 * 【说明】accept 线程：像"前台接线员"，只管接电话，不处理对话。每接到一个客户端：
 *    accept → 检查人数上限 → 设置发送超时 → malloc 包裹 ctx → 计数 +1 → 创建客户端线程
 *   ★计数必须在 pthread_create 之前 +1★：否则新线程可能比 +1 更早结束 -1，计数出现负数/错乱。
 *   创建失败要把前面做的事全部撤销（计数 -1、free ctx、close fd）。
 *   accept(监听fd, &客户端地址, &地址长度)：取出一个已完成握手的连接，返回新的 fd（"专线"）。
 */
static void *rtsp_server_thread(void *arg)
{
    rtsp_server *server = (rtsp_server *)arg;

    while (atomic_load(&server->running)) {
        struct sockaddr_in client_addr;             /* accept 会把客户端的地址填进来 */
        socklen_t addr_len = sizeof(client_addr);   /* 每轮都要重新设置：accept 会改写它 */
        rtsp_client_context *ctx;
        pthread_attr_t attr;
        pthread_t tid;
        int client_fd;
        int ret;

        /* 先等总机可读，最多 1 秒，避免卡在 accept 里无法退出 */
        ret = wait_readable(server->listen_fd, 1000);
        if (ret < 0) {
            printf("rtsp: select failed: %s\n", strerror(errno));
            break;
        }
        if (ret == 0) {
            /* 超时，回去检查 running */
            continue;
        }

        /* 监听 fd 可读，说明有新连接在排队，accept 把它取出来，返回专线 client_fd */
        client_fd = accept(server->listen_fd, (struct sockaddr *)&client_addr, &addr_len);
        if (client_fd < 0) {
            /* 失败多半是暂时的（例如对方刚连上就断开），打印后继续等，不退出线程 */
            printf("rtsp: accept failed: %s\n", strerror(errno));
            continue;
        }
        /* 客户端太多：直接断开 */
        if (atomic_load(&server->active_clients) >= RTSP_MAX_CLIENTS) {
            printf("rtsp: too many clients (max %d), reject\n", RTSP_MAX_CLIENTS);
            close(client_fd);
            continue;
        }

        /* 发送超时 3 秒：对方一直不收数据时，send 不会永远卡住 */
        {
            struct timeval snd_timeout = { 3, 0 };
            /*
             * 【说明】setsockopt：设置 socket 选项。SO_SNDTIMEO = 发送超时。
             *   如果客户端不再读取数据（比如 VLC 卡死），内核发送缓冲区被填满后，send 默认会永远阻塞；
             *   设置 3 秒超时后，send 会返回错误，我们就能结束这个会话，不会卡死线程。
             *   recv 这边不需要设接收超时：我们先用 select（wait_readable）确认有数据才 recv，本来就不会无限阻塞。
             */
            setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &snd_timeout, sizeof(snd_timeout));
        }
        ctx = (rtsp_client_context *)malloc(sizeof(*ctx));
                if (ctx == NULL) {
            printf("rtsp: malloc failed\n");
            close(client_fd);
            continue;
        }
        ctx->server    = server;
        ctx->client_fd = client_fd;
        ctx->addr      = client_addr;               /* 整个结构体拷贝 */

        atomic_fetch_add(&server->active_clients, 1);   /* 客户端计数 +1 */

        pthread_attr_init(&attr);
        /*
         * 【说明】detached（分离）线程：线程结束时系统自动回收它的资源，不需要别人 pthread_join。
         *   代价是：没法用 join 等它结束，所以才需要 active_clients 计数器 + stop 里的等待循环，
         *   来确认"所有客户端线程都已经退出"。
         *   pthread_attr_t 是线程"属性"对象：init → 设置各项 → 传给 pthread_create → destroy。
         */
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);   /* 线程结束后自动释放资源 */
        pthread_attr_setstacksize(&attr, RTSP_CLIENT_STACK_SIZE);       /* 线程*/
        ret = pthread_create(&tid, &attr, rtsp_client_thread, ctx);
        pthread_attr_destroy(&attr);

        if (ret != 0) {
            /* 创建失败：把前面做的事撤销 */
            printf("rtsp: pthread_create failed\n");
            atomic_fetch_sub(&server->active_clients, 1);
            free(ctx);
            close(client_fd);
        }
    }

    return NULL;
}

/* 启动 RTSP 服务器，监听指定端口 */
/*
 * 【说明】启动 = 网络编程"五步曲"：socket → setsockopt(SO_REUSEADDR) → bind → listen → 启动 accept 线程。
 *    socket(AF_INET, SOCK_STREAM, 0)  创建 TCP socket
 *    bind(fd, 地址, 长度)             把 socket 绑定到本机某个端口（端口被占用会失败）
 *    listen(fd, backlog)              转为"监听"状态，从此可以 accept 新连接
 *   INADDR_ANY = 0.0.0.0，表示本机所有网卡；htons/htonl 把端口/地址转成网络字节序（大端）。
 *   网络初始化全部成功后，最后才创建线程（先准备好再使用）。
 */
rtsp_server *rtsp_server_start(unsigned short port)
{
    rtsp_server *server;        /* RTSP 服务器上下文 */
    struct sockaddr_in addr;    /* 本地监听地址 */
    int on = 1;                 /* SO_REUSEADDR 的开关值 */
    int ret;

    /* 分配并清零服务器上下文 */
    server = (rtsp_server *)calloc(1, sizeof(*server));
    if (server == NULL) {
        printf("rtsp: malloc failed\n");
        return NULL;
    }

    /*
     * 1. 创建 TCP socket。
     *    SOCK_STREAM = TCP；RTP 用的是 SOCK_DGRAM = UDP。
     */
    server->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server->listen_fd < 0) {
        perror("rtsp: socket");
        goto free_server;
    }

    /*
     * 2. 允许程序重启后立即重新使用相同端口。
     *    否则旧连接关闭后，端口会处于 TIME-WAIT 状态一段时间，
     *    这期间 bind 会报 "Address already in use"。
     */
    ret = setsockopt(server->listen_fd, SOL_SOCKET, SO_REUSEADDR,
                     &on, sizeof(on));
    if (ret < 0) {
        perror("rtsp: setsockopt SO_REUSEADDR");
        goto close_fd;
    }

    /*
     * 3. 初始化监听地址：
     *    - AF_INET：IPv4
     *    - port：监听端口，转换为网络字节序
     *    - INADDR_ANY：监听本机所有网卡，不管客户端从哪个 IP 连进来都接受
     */
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    /* 4. 将 socket 绑定到指定端口；端口已被其他程序占用时会失败 */
    ret = bind(server->listen_fd,
               (struct sockaddr *)&addr,
               sizeof(addr));
    if (ret < 0) {
        perror("rtsp: bind");
        goto close_fd;
    }

    /*
     * 5. 开始监听客户端连接，此后它就是"总机"。
     *    backlog 为 4：内核完成握手后，最多允许 4 个连接在排队区等待 accept。
     */
    ret = listen(server->listen_fd, 4);
    if (ret < 0) {
        perror("rtsp: listen");
        goto close_fd;
    }
    /*
     * 6. 网络初始化全部成功后，最后才启动 RTSP 线程。
     *    这样线程一启动，用到的 socket 就已经准备好了。
     */
    atomic_store(&server->running, 1);
    
    ret = pthread_create(&server->thread, NULL,
                         rtsp_server_thread, server);
    if (ret != 0) {
        printf("rtsp: pthread_create failed\n");
        goto close_fd;
    }

    printf("rtsp: listening on port %u\n", (unsigned)port);
    return server;

/* socket 创建成功但后续初始化失败时，关闭 socket */
close_fd:
    close(server->listen_fd);

/* 释放服务器上下文 */
free_server:
    free(server);
    return NULL;
}

/* 停止 RTSP 服务器：顺序与 start 相反 */
/*
 * 【说明】停止顺序（与 start 相反，务必保持）：
 *    1. running = 0           通知所有线程该退出了
 *    2. pthread_join(accept线程)  等 accept 线程退出 → 之后不会再有新客户端
 *    3. 等 active_clients == 0   等所有客户端线程退出（它们是 detached，无法 join，只能靠计数器）
 *    4. close(listen_fd)        关监听 socket
 *    5. free(server)            最后才能释放服务器对象
 *   若某一步提前，就会出现"线程还在用 server，却已经被 free"的崩溃。
 *   usleep(10*1000) = 睡 10 毫秒（微秒单位），避免空转占满 CPU。
 */
void rtsp_server_stop(rtsp_server *server)
{
    /* 允许传入 NULL，避免空指针解引用 */
    if (server == NULL) {
        return;
    }

    /*
     * 通知线程退出。
     * 线程最多 1 秒就会从 wait_readable 返回并检查到 running 为 0。
     */
    atomic_store(&server->running, 0);

    /* 等待线程完全退出，确保没有代码再访问 server 和 listen_fd */
    pthread_join(server->thread, NULL);
    /* 此时不会再有新客户端，但已有的客户端线程还在运行，等它们全部结束 */
    while (atomic_load(&server->active_clients) > 0) {
        usleep(10 * 1000);      /* 睡 10 毫秒再查 */
    }
    /* 线程退出后再关闭监听 socket */
    close(server->listen_fd);

    /* 释放服务器上下文 */
    free(server);

    printf("rtsp: stopped\n");
}
