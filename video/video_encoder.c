/*
 * 【说明】本文件的函数已按"实际使用顺序"排好，从上往下读就是整个编码器的生命周期：
 *   ① 启动阶段：create → start_receive → bind_vpss（配置编码通道，并和 VPSS 接上）
 *   ② 运行阶段：video_encoder_save_one_frame（真正"取一帧码流"，最重要，务必读懂）
 *               → video_encoder_stream_thread（反复调用上一个）→ start_stream / stop_stream（开/关这个线程）
 *   ③ 退出阶段：get_stream_result（查结果）→ unbind_vpss → stop_receive → destroy（启动的反序）
 *
 *   【数据走向】
 *      VPSS ──bind──► VENC 通道 0 ──(编码完成后)──► video_encoder_save_one_frame 取走
 *                                                    ├─► 写入 stream.h264 文件（调试用）
 *                                                    ├─► nal_cache_sps_pps：缓存 SPS/PPS（给 SDP 用）
 *                                                    └─► media_source_push：分发给所有订阅者(RTP 发送等)
 */

/*
 * @file video_encoder.c
 * @brief 实现VPSS到VENC的H.264编码链路及后台码流保存。
 *
 * 取流线程负责从编码器获取码流，主线程负责启动、停止线程
 * 以及在确认线程退出后关闭输出文件。
 */
#include <stdio.h>

#include "app_config.h"
#include "video_encoder.h"
#include "ss_mpi_venc.h"
#include "ss_mpi_sys_bind.h"
#include <stdlib.h>
#include <sys/select.h>
#include "media_source.h"
#include "media_source.h"
#include "nal_parser.h"

#include <pthread.h>
#include <string.h>
#include <stdbool.h>
#include <stdatomic.h>

/* 取流线程标识 */
/*
 * 【说明】下面这一组 g_stream_xxx 全局变量，共同管理"取流线程"的生命周期：
 *   g_stream_thread           线程 id（pthread_join 时要用）
 *   g_stream_thread_created   线程是否创建成功（只有成功才可以 join）
 *   g_stream_file             输出的 stream.h264 文件指针；必须等线程结束后才能 fclose
 *   g_stream_stop_requested   "请你停下"的标志：主线程写，取流线程读。
 *                             因为是两个线程同时访问，所以用 atomic_bool（原子变量），
 *                             读写它不会被编译器优化掉，也不会读到"写了一半"的值。
 *   g_stream_result           线程运行的结果，线程退出后主线程再去读
 *   g_stream_frames           已经处理的帧数（统计用）
 */
static pthread_t g_stream_thread;

/* 记录线程是否已经创建，由主线程维护 */
static td_bool g_stream_thread_created = TD_FALSE;

/* 输出文件，在线程结束后才能关闭 */
static FILE *g_stream_file = NULL;

/* 主线程设置、取流线程读取的退出标志 */
static atomic_bool g_stream_stop_requested = ATOMIC_VAR_INIT(false);

/* 取流线程的执行结果，主线程在线程结束后读取 */
static td_s32 g_stream_result = TD_SUCCESS;

/* 已经保存的编码帧数 */
static td_u32 g_stream_frames = 0;

/* 拼帧缓冲：把一帧的多个pack拼成连续码流后再投递 */
/*
 * 【说明】"拼帧缓冲"（可以想成一个反复使用的工作台，512 KB）：
 *   编码器给的一帧可能分成多个 pack（碎片），比如 [SPS][PPS][IDR 切片]。
 *   下游（RTP 打包、录像）希望拿到"一帧完整连续的数据"，所以先把所有 pack 拷贝拼到这里。
 *   它是 static 全局数组，只被取流线程这一个线程使用，所以不需要加锁。
 *   media_source_push 会再把这块数据复制一份给每个订阅者，所以这里可以放心下一帧重复使用。
 */
static td_u8 g_frame_buf[MEDIA_FRAME_MAX_SIZE];

/* 创建并配置视频编码器通道 */
/*
 * 【说明】创建 H.264 编码通道，参数分四组（都放在 ot_venc_chn_attr 里）：
 *
 *   ① venc_attr：编码本身
 *      - type OT_PT_H264         ：编码标准 H.264（PT = Payload Type，载荷类型）
 *      - max_pic_* / pic_*       ：最大/实际编码尺寸 1280x720
 *      - buf_size                ：编码器输出码流缓冲区的字节数（估算值：宽x高x3/4，按 64 字节对齐）
 *      - profile = 0             ：Baseline Profile（基础档）。特点：兼容性最好，几乎所有播放器和
 *                                  手机都能解；没有 B 帧，延迟低。这也是 SDP 里 profile-level-id=42xxxx 里的 "42"。
 *      - is_by_frame = TRUE      ：一次取流拿到"一整帧"，而不是半帧（网络发送按帧处理更方便）
 *      - rcn_ref_share_buf_en    ：重建/参考帧缓冲共享，关闭
 *      - frame_buf_ratio = 75    ：帧缓冲占比，沿用 SDK 示例
 *
 *   ② rc_attr（Rate Control，码率控制）：
 *      - OT_VENC_RC_MODE_H264_CBR：CBR = Constant Bit Rate，恒定码率。
 *          码率稳定、好估算存储容量/网络带宽（适合网络直播、录像），代价是复杂画面可能不够清晰。
 *          另一种 VBR（可变码率）画质更均匀但码率有波动。
 *      - bit_rate = 2048 kbps    ：目标码率 2 Mbit/s（见 app_config.h）
 *      - gop = 帧率x2 = 60       ：GOP(Group Of Pictures)=相邻两个 I 帧之间的帧数。
 *                                  30 fps 下就是每 2 秒一个 I 帧(关键帧/IDR)。
 *                                  I 帧可独立解码，播放器要等到收到 I 帧才能开始显示，
 *                                  所以 GOP 越大，客户端连上后等待出图越久（最长 2 秒）。
 *      - stats_time = 1          ：码率统计的时间窗口 1 秒
 *      - src_frame_rate / dst_frame_rate：输入帧率 / 输出帧率，相同则不丢帧
 *
 *   ③ gop_attr：GOP 结构
 *      - OT_VENC_GOP_MODE_NORMAL_P：普通 P 帧模式 = I P P P P ... （没有 B 帧），
 *        P 帧只参考前一帧，延迟最低，最适合实时监控。
 *      - ip_qp_delta = 0         ：I 帧与 P 帧量化参数差，0 = 默认。
 */
td_s32 video_encoder_create(void)
{
    td_s32 ret;
    ot_venc_chn venc_chn = 0;

    /* 配置编码格式、图像尺寸、码率控制和GOP参数 */
    ot_venc_chn_attr chn_attr={
        /*VENC相关属性*/
        .venc_attr = {
            .type = OT_PT_H264,
            .max_pic_width = APP_VPSS_OUT_WIDTH,
            .max_pic_height = APP_VPSS_OUT_HEIGHT,
            .pic_width = APP_VPSS_OUT_WIDTH,
            .pic_height = APP_VPSS_OUT_HEIGHT,
            /* 沿用SDK样例的码流缓存估算，并按64字节对齐 */
            .buf_size = ((APP_VPSS_OUT_WIDTH * APP_VPSS_OUT_HEIGHT * 3 / 4 + 63) / 64)*64,
            /* H.264 Baseline Profile*/
            .profile = 0,

            /* 按帧获取编码码流 */
            .is_by_frame = TD_TRUE,

            /*h264相关属性*/
            .h264_attr = {
                .rcn_ref_share_buf_en = TD_FALSE,
                .frame_buf_ratio = 75,
            },

        },
        /* 码率控制属性 */
        .rc_attr = {
            .rc_mode = OT_VENC_RC_MODE_H264_CBR,

            .h264_cbr = {
                .gop = APP_VIDEO_FPS*2,
                .stats_time = 1,

                .src_frame_rate = APP_VIDEO_FPS,
                .dst_frame_rate = APP_VIDEO_FPS,

                .bit_rate = APP_VENC_BITRATE_KBPS,
            },
        },
        /* GOP属性 */
        .gop_attr = {
            .gop_mode = OT_VENC_GOP_MODE_NORMAL_P,

            .normal_p = {
                .ip_qp_delta = 0,
            },
        },
    };
    /* 向媒体SDK创建编码通道 */
    /*
     * 【说明】ss_mpi_venc_create_chn(通道号, &属性)：创建编码通道。以后对这个通道的操作都用 venc_chn=0 指代它。
     */
    ret = ss_mpi_venc_create_chn(venc_chn, &chn_attr);
    if (ret != TD_SUCCESS) {
        printf("ss_mpi_venc_create_chn failed with %#x!\n", ret);
        return ret;
    }

    return TD_SUCCESS;
}

/* 启动编码通道0，持续接收图像 */
/*
 * 【说明】让编码通道开始接收图像并编码。
 *   recv_pic_num = -1 表示"一直接收、永不自动停止"；填正数 N 表示只编码 N 帧后自动停。
 *   注意先 start_receive，再 bind_vpss：这样 VPSS 一开始送图，编码器就已经准备好接收了。
 */
td_s32 video_encoder_start_receive(void)
{
    td_s32 ret;
    ot_venc_chn venc_chn = 0;

    ot_venc_start_param start_param = {
        .recv_pic_num = -1, /* 持续接收图像 */
    };

    ret = ss_mpi_venc_start_chn(venc_chn, &start_param);
    if (ret != TD_SUCCESS) {
        printf("ss_mpi_venc_start_recv_frame failed with %#x!\n", ret);
        return ret;
    }

    printf("video_encoder_start_receive success!\n");
    return TD_SUCCESS;
}

/* 将VPSS输出绑定到VENC输入，使图像自动送入编码器 */
/*
 * 【说明】把 VPSS Grp0/Chn0 的输出口接到 VENC Chn0 的输入口（原理见 video_processing.c 的 bind 说明）。
 *   绑定成功的这一刻，图像就开始流向编码器，后面取流线程就能取到码流了。
 */
td_s32 video_encoder_bind_vpss(void)
{
    td_s32 ret;
    ot_mpp_chn src = {
        .mod_id = OT_ID_VPSS,
        .dev_id = 0,
        .chn_id = 0,
    };
    ot_mpp_chn dst = {
        .mod_id = OT_ID_VENC,
        .dev_id = 0,
        .chn_id = 0,
    };

    /* 建立VPSS Channel 0到VENC Channel 0的数据通路 */
    ret = ss_mpi_sys_bind(&src, &dst);
    if (ret != TD_SUCCESS) {
        printf("ss_mpi_sys_bind failed with %#x!\n", ret);
        return ret;
    }

    printf("video_encoder_bind_vpss success!\n");
    return TD_SUCCESS;
}

/* 获取并保存一帧编码码流 */
/*
 * 【说明】★本函数是"编码器 → 网络"的入口，务必读懂★
 *   做什么：从编码器取走"一帧"码流，写到文件，同时把它分发给 media_source 的所有订阅者。
 *
 *   使用的 SDK 函数：
 *    ss_mpi_venc_get_fd(chn)         ：拿到这个编码通道的"文件描述符"，用于 select 等待
 *    select(...)                     ：Linux 的"等待多个文件描述符就绪"函数。这里只等 1 个：
 *                                       编码器有新码流时，这个 fd 就变成"可读"，select 立刻返回；
 *                                       没有就最多阻塞 3 秒（避免死循环空转吃满 CPU）。
 *    ss_mpi_venc_query_status        ：查询当前通道状态，我们要它的 cur_packs（这一帧有几个 pack）
 *    ss_mpi_venc_get_stream          ：取走这一帧；注意它给的是"借"来的内存，
 *                                       数据仍在 SDK 的内部缓冲里，我们只拿到了指针
 *    ss_mpi_venc_release_stream      ：用完必须还！不还的话编码器缓冲区会被占满，最后编码卡死
 *
 *   流程：select 等待 → query_status 得到 pack 数 → calloc 分配 pack 描述数组
 *        → get_stream 取流 → 循环把每个 pack 拼进 g_frame_buf 并写文件
 *        → nal_cache_sps_pps + media_source_push 分发 → release_stream 归还 → free 数组
 */
td_s32 video_encoder_save_one_frame(FILE *file)
{
    td_s32 ret;
    td_s32 release_ret;
    ot_venc_chn venc_chn = 0;

    /*
     * 【说明】fd_set / FD_ZERO / FD_SET / struct timeval：都是 select 的配套工具。
     *    fd_set        ：一个"文件描述符集合"（位图）
     *    FD_ZERO(&s)   ：清空集合
     *    FD_SET(fd,&s) ：把 fd 加入集合（表示"我要等它"）
     *    timeval       ：超时时间（秒+微秒）
     *    FD_SETSIZE    ：fd_set 最多能容纳的 fd 编号上限（通常 1024），fd 太大放不进去，所以要检查。
     */
    int venc_fd;
    int ready;
    fd_set read_fds;
    struct timeval timeout;

    /* status用于查询当前可用的pack数量，stream保存当前帧数据 */
    ot_venc_chn_status status = {0};
    ot_venc_stream stream = {0};

    td_u32 frame_len = 0;
    /* 输出文件由调用方管理，本函数只负责写入 */
    if (file == NULL) {
        printf("Output file is NULL\n");
        return TD_FAILURE;
    }

    /* 1. 获取编码通道对应的文件描述符 */
    venc_fd = ss_mpi_venc_get_fd(venc_chn);
    if (venc_fd < 0 || venc_fd >= FD_SETSIZE) {
        printf("Invalid VENC file descriptor: %d\n", venc_fd);
        return TD_FAILURE;
    }

    /* 2. 等待码流就绪，最多阻塞3秒 */
    FD_ZERO(&read_fds);
    FD_SET(venc_fd, &read_fds);

    timeout.tv_sec = 3;
    timeout.tv_usec = 0;

    /*
     * 【说明】select 的返回值：  <0 出错 / =0 超时 / >0 有 fd 就绪。
     *   注意第一个参数是"最大 fd 编号 + 1"（不是 fd 个数），这是 select 的历史规定。
     *   超时返回 OT_ERR_VENC_BUF_EMPTY，让线程"再等一轮"而不是当作错误。
     */
    ready = select(
        venc_fd + 1,
        &read_fds,
        NULL,
        NULL,
        &timeout);

    if (ready < 0) {
        perror("Wait for VENC stream failed");
        return TD_FAILURE;
    }

    if (ready == 0) {
        printf("Wait for VENC stream timed out\n");
        return OT_ERR_VENC_BUF_EMPTY;
    }

    /* 3. 查询当前待取帧包含的pack数量 */
    /*
     * 【说明】status.cur_packs = 当前待取的这一帧由多少个 pack 组成。
     *   pack（码流包）是编码器输出的最小单位，一帧可以含多个：
     *     IDR 帧常见：[SPS][PPS][SEI?][IDR 切片…]   P 帧常见：[P 切片]
     *   所以需要先知道个数，才能分配对应大小的数组去接收它们的描述信息。
     */
    ret = ss_mpi_venc_query_status(venc_chn, &status);
    if (ret != TD_SUCCESS) {
        printf("Query VENC status failed: %#x\n", (td_u32)ret);
        return ret;
    }

    if (status.cur_packs == 0) {
        printf("No VENC packs available\n");
        return OT_ERR_VENC_BUF_EMPTY;
    }

    /* 4. 为SDK返回的pack描述信息分配数组 */
    /*
     * 【说明】calloc(个数, 每个大小)：分配并清零一块堆内存（比 malloc 多清零）。
     *   这里的 pack 数组只是"描述信息"（每个 pack 的地址、长度、偏移、pts），不是码流数据本身。
     *   必须在函数每个出口都 free（见最后的 free(stream.pack) 和 get_stream 失败分支）。
     *   （优化想法：每帧都 calloc/free 有点浪费，以后可改成固定大小的静态数组。）
     */
    stream.pack = calloc(
        status.cur_packs, sizeof(ot_venc_pack));

    if (stream.pack == NULL) {
        printf("Allocate VENC pack array failed\n");
        return TD_FAILURE;
    }

    stream.pack_cnt = status.cur_packs;

    /* 5. 从编码器获取这一帧的码流，最多等待1000毫秒 */
    /*
     * 【说明】get_stream(通道, &stream, 超时毫秒)：取流。
     *   调用前 stream.pack 必须指向我们分配好的数组、stream.pack_cnt 填好个数。
     *   成功后每个 pack 里：addr(数据起始地址)、len(长度)、offset(有效数据距 addr 的偏移)、pts(时间戳，微秒)。
     *   从这里开始到 release_stream 之前，stream 里的指针都是"借来的"，一定要还。
     *   下面所有失败路径都用 goto release_stream，保证"借了就一定还"。
     */
    ret = ss_mpi_venc_get_stream(venc_chn, &stream, 1000);
    if (ret != TD_SUCCESS) {
        printf("Get VENC stream failed: %#x\n", (td_u32)ret);
        free(stream.pack);
        return ret;
    }

    /* 6. 按顺序写入这一帧的所有pack数据 */
    for (td_u32 i = 0; i < stream.pack_cnt; i++) {
        const ot_venc_pack *pack = &stream.pack[i];
        td_u32 data_size;

        if (pack->addr == NULL || pack->offset > pack->len) {
            printf("Invalid VENC pack information\n");
            ret = TD_FAILURE;
            goto release_stream;
        }

        /* 跳过pack前面的偏移区域，只写入有效码流数据 */
        /*
         * 【说明】有效数据 = 从 (addr + offset) 开始，共 len - offset 个字节。
         *   offset 是 SDK 在 pack 头部留的空隙（例如对齐用），不属于码流，必须跳过。
         */
        data_size = pack->len - pack->offset;

        /*如果拼帧缓冲区还有空间*/
        /*
         * 【说明】边拼边检查，防止一帧太大（超过 g_frame_buf 的 512 KB）时写穿数组导致内存损坏。
         *   2 Mbit/s 码率下 I 帧通常几十 KB，远小于 512 KB；若以后码率或分辨率提高很多，需要调大
         *   media_source.h 里的 MEDIA_FRAME_MAX_SIZE。
         */
        if(frame_len + data_size <= MEDIA_FRAME_MAX_SIZE)
        {
            memcpy(g_frame_buf + frame_len, pack->addr + pack->offset, data_size);
            frame_len += data_size;
        }

        else{
            printf("Frame size exceeds buffer limit\n");
            ret = TD_FAILURE;
            goto release_stream;
        }

        /*
         * 【说明】⚠ 写调试文件 stream.h264：
         *   注意它排在 media_source_push 之前，且写失败会 goto release_stream 直接跳过分发。
         *   也就是说：如果 SD 卡写满/NFS 断开导致 fwrite 失败，RTSP 画面也会跟着断掉，
         *   在做正式的录像功能时，要把"写文件失败"和"网络推流"解耦。
         */
        if (data_size > 0 &&
            fwrite(pack->addr + pack->offset,
                   1, data_size, file) != data_size) {
            printf("Write H.264 stream failed\n");
            ret = TD_FAILURE;
            goto release_stream;
        }
    }

    /*pack拼完了，给media来分发给所有消费者*/
    /*
     * 【说明】分发阶段（拼帧完成后）：
     *    nal_cache_sps_pps(buf, len) ：扫描这一帧里有没有 SPS/PPS，有就缓存起来。
     *                                   SPS/PPS 只在 IDR 帧附近出现（约每 2 秒一次），
     *                                   但客户端 DESCRIBE 随时可能到来，所以要先缓存起来备用。
     *    media_source_push(...)      ：把整帧交给 media_source，由它复制给每个订阅者
     *                                   （调试消费者、每个 RTSP 客户端的 rtp_sender 等）。
     *                                   参数 stream.pack[0].pts = 这一帧的时间戳(微秒)；
     *                                   TD_TRUE 表示这是视频帧。
     *    (void) 强转：表示"我知道它有返回值，但这里故意忽略"，也消除编译警告。
     */
    if (frame_len > 0) {
        nal_cache_sps_pps(g_frame_buf, frame_len);

        (void)media_source_push(g_frame_buf, frame_len,
                                stream.pack[0].pts, TD_TRUE);
    }

/*
 * 【说明】release_stream 标签：无论上面成功还是失败，最终都汇合到这里把借来的码流还给 SDK，
 *   然后 free 掉我们自己分配的 pack 数组。这就是 C 里"单出口清理"的标准写法。
 */
release_stream:
    /*
     * 7. 获取成功后，无论写文件是否成功，
     * 都需要把借用的码流交还给SDK。
     */
    release_ret = ss_mpi_venc_release_stream(venc_chn, &stream);
    if (release_ret != TD_SUCCESS) {
        printf("Release VENC stream failed: %#x\n",
               (td_u32)release_ret);

        /* 如果之前没有错误，返回释放码流的错误 */
        if (ret == TD_SUCCESS) {
            ret = release_ret;
        }
    }

    /* 8. 释放应用自己分配的pack描述数组 */
    free(stream.pack);

    return ret;
}

/* 取流线程：持续获取编码码流并写入输出文件 */
/*
 * 【说明】取流线程：不断循环，调用 save_one_frame 取一帧、处理一帧。
 *   - 返回 OT_ERR_VENC_BUF_EMPTY（暂时没码流）：继续循环再等，不算错误
 *   - 返回其他错误：记录到 g_stream_result 并退出循环
 *   - 循环条件 !atomic_load(&stop_requested)：主线程设置 true 后，线程在下一轮检查时就会退出
 *   线程每轮最多在 select 里等 3 秒（见 save_one_frame），所以停止请求最迟 3 秒内会被响应。
 */
static void *video_encoder_stream_thread(void *arg)
{
    td_s32 ret;

    (void)arg; /* 避免未使用参数的编译警告 */

    printf("video_encoder_stream_thread started\n");

    /* 在线程收到停止请求前，持续处理编码码流 */
    while(!atomic_load(&g_stream_stop_requested)) {
        ret = video_encoder_save_one_frame(g_stream_file);

        /* 暂时没有码流，回到循环继续等待 */
        if (ret == OT_ERR_VENC_BUF_EMPTY) {
            continue;
        }

        if (ret != TD_SUCCESS) {
            printf("video_encoder_save_one_frame failed with %#x!\n", ret);
            g_stream_result = ret;
            break;
        }
        /* 成功写入一帧后更新统计信息 */
        g_stream_frames++;

        /* 第一帧及之后每30帧打印一次进度 */
        /*
        if (g_stream_frames == 1 || g_stream_frames % 30 == 0) {
            printf("H.264 saved frames: %u\n", g_stream_frames);
        }
        */

    }

    printf("video_encoder_stream_thread stopped\n");
    return NULL;
}

/* 打开输出文件并启动后台取流线程 */
/*
 * 【说明】启动取流：先打开文件，再创建线程（顺序很重要：线程一启动就会用到文件）。
 *   fopen(路径, "wb")：以"写+二进制"方式打开，文件不存在则创建，存在则清空覆盖。
 *   ⚠ 这个 stream.h264 是调试用的，会一直追加写，不限大小。码率 2 Mbit/s 时约每小时 900 MB，
 *     SD 卡会被写满。你已决定后面再做容量限制，这里先留个记号：以后录像功能（MP4 分段、循环覆盖）
 *     做好后，应该把这个调试文件关掉或限制大小。
 */
td_s32 video_encoder_start_stream(const char *file_path)
{
    td_s32 thread_ret;

    if (file_path == NULL) {
        printf("H.264 output path is NULL\n");
        return TD_FAILURE;
    }

    /* 防止重复启动线程或复用尚未关闭的输出文件 */
    if(g_stream_thread_created || g_stream_file != NULL) {
        printf("Stream thread is already running\n");
        return TD_FAILURE;
    }

    /* 以二进制写模式创建或覆盖输出文件 */
    g_stream_file = fopen(file_path, "wb");
    if (g_stream_file == NULL) {
        perror("Failed to open output file");
        return TD_FAILURE;
    }

    /* 初始化本次取流任务的状态 */
    g_stream_frames = 0;
    g_stream_result = TD_SUCCESS;
    atomic_store(&g_stream_stop_requested, false);

    /* 创建后台线程，实际取流工作在线程中执行 */
    thread_ret = pthread_create(&g_stream_thread, NULL, video_encoder_stream_thread, NULL);
    if (thread_ret != 0) {
        printf("Failed to create stream thread: %s\n", strerror(thread_ret));
        /* 线程未创建成功，直接关闭已经打开的文件 */
        if (fclose(g_stream_file) != 0) {
            perror("Cleanup: close H.264 file failed");
        }

        g_stream_file = NULL;
        return TD_FAILURE;
    }

    g_stream_thread_created = TD_TRUE;
    printf("video_encoder_start_stream success, output file: %s\n", file_path);

    return TD_SUCCESS;
}

/* 请求取流线程退出，等待结束后关闭文件 */
/*
 * 【说明】停止顺序（经典的"先通知、再等待、最后释放"三步）：
 *    ① atomic_store(stop_requested, true)  ：通知线程该停了
 *    ② pthread_join                        ：阻塞等待线程真正退出
 *    ③ fclose(文件)                        ：线程不再用文件了，才可以安全关闭
 *   顺序不能反：如果先 fclose，线程还在 fwrite，就会写到一个已关闭的文件指针上，程序崩溃。
 *   fclose 的返回值要检查：缓冲区里没写完的数据是在 fclose 时才真正落盘的，可能此时才发现磁盘满。
 */
td_s32 video_encoder_stop_stream(void)
{
    int thread_ret;

    /* 没有创建线程，或者已经停止 */
    if (g_stream_thread_created == TD_FALSE) {
        return TD_SUCCESS;
    }

    /* 1. 通知取流线程结束循环 */
    atomic_store(&g_stream_stop_requested, true);

    /* 2. 等待线程完成当前操作并退出 */
    thread_ret = pthread_join(g_stream_thread, NULL);
    if (thread_ret != 0) {
        printf("Join VENC stream thread failed: %s\n",
               strerror(thread_ret));

        /* 无法确认线程已经结束，保留文件和状态 */
        return TD_FAILURE;
    }

    g_stream_thread_created = TD_FALSE;

    /* 线程已经结束，现在可以读取它的执行结果 */
    if (g_stream_frames == 0) {
        printf("No H.264 frames were saved\n");
        g_stream_result = TD_FAILURE;
    }

    /* 3. 线程不再使用文件，可以关闭并检查写出结果 */
    if (g_stream_file != NULL) {
        if (fclose(g_stream_file) != 0) {
            perror("Close H.264 file failed");
            g_stream_result = TD_FAILURE;
        }

        g_stream_file = NULL;
    }

    printf("VENC stream stopped, saved frames=%u\n",
           g_stream_frames);

    /* 表示线程和文件的清理已经完成 */
    return TD_SUCCESS;
}

/* 在线程结束后，查询取流和文件写入是否成功 */
/*
 * 【说明】取流线程结束后，查询它整个运行期间是否出过错（main.c 退出时用它判断录像是否成功）。
 *   线程还在运行时不允许读（会和线程同时读写 g_stream_result），所以先检查线程已回收。
 */
td_s32 video_encoder_get_stream_result(void)
{
    /* 线程未回收时，不读取它正在修改的结果 */
    if (g_stream_thread_created) {
        printf("Stop VENC stream before querying its result\n");
        return TD_FAILURE;
    }

    /* 返回线程记录的取流或文件写入结果 */
    return g_stream_result;
}

/* 解除VPSS Group 0 Channel 0到VENC Channel 0的绑定 */
td_s32 video_encoder_unbind_vpss(void)
{
    td_s32 ret;
    ot_mpp_chn src = {
        .mod_id = OT_ID_VPSS,
        .dev_id = 0,
        .chn_id = 0,
    };
    ot_mpp_chn dst = {
        .mod_id = OT_ID_VENC,
        .dev_id = 0,
        .chn_id = 0,
    };

    /* 拆除VPSS与VENC之间的数据通路 */
    ret = ss_mpi_sys_unbind(&src, &dst);
    if (ret != TD_SUCCESS) {
        printf("ss_mpi_sys_unbind failed with %#x!\n", ret);
        return ret;
    }

    printf("video_encoder_unbind_vpss success!\n");
    return TD_SUCCESS;
}

/* 停止编码通道0接收图像 */
/*
 * 【说明】停止接收。main.c 清理时的顺序：stop_stream(取流线程) → unbind_vpss → stop_receive → destroy。
 */
td_s32 video_encoder_stop_receive(void)
{
    td_s32 ret;
    ot_venc_chn venc_chn = 0;

    ret = ss_mpi_venc_stop_chn(venc_chn);
    if (ret != TD_SUCCESS) {
        printf("ss_mpi_venc_stop_recv_frame failed with %#x!\n", ret);
        return ret;
    }

    printf("video_encoder_stop_receive success!\n");
    return TD_SUCCESS;
}

/* 销毁编码通道0 */
/*
 * 【说明】销毁编码通道。必须先 stop_receive（停止接收），再 destroy；否则会返回忙/失败。
 */
td_s32 video_encoder_destroy(void)
{
    td_s32 ret;
    ot_venc_chn venc_chn = 0;

    ret = ss_mpi_venc_destroy_chn(venc_chn);
    if (ret != TD_SUCCESS) {
        printf("ss_mpi_venc_destroy_chn failed with %#x!\n", ret);
        return ret;
    }

    printf("video_encoder_destroy success!\n");
    return TD_SUCCESS;
}
