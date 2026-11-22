/*
 * 【说明】本文件是整个项目里"最像硬件初始化"的部分，函数很多，但套路只有一个：
 *     open 设备 → 填一个"属性结构体" → 调 ioctl / ss_mpi_xxx 提交 → 失败就回滚
 *   所以只要看懂下面几类调用，就能看懂全文件：
 *
 *   (1) open / close / ioctl —— Linux 驱动的标准用法
 *       open("/dev/ot_mipi_rx", O_RDWR) ：打开一个"设备文件"。在 Linux 里硬件驱动会把自己
 *                                          伪装成 /dev 下的文件，我们通过它跟驱动说话。
 *       ioctl(fd, 命令, &参数)          ：给驱动发一条"命令"，并把一个结构体当参数传进去。
 *                                          命令是 OT_MIPI_xxx 这类常量。返回负数表示失败。
 *       close(fd)                        ：关闭设备文件。注意：关闭文件 ≠ 关闭硬件，
 *                                          硬件状态是被 ioctl 命令改变的，close 不会撤销它们。
 *       perror("xxx")                    ：把"xxx: 系统错误原因"打印到屏幕（比如 No such file）。
 *
 *   (2) ss_mpi_vi_xxx —— 海思闭源 VI 库的函数，直接传"编号+属性结构体"即可，不需要 open。
 *
 *   (3) goto 清理 —— 每步失败就跳到对应标签，把前面已经成功的步骤倒着撤销。
 *       这是 C 语言里管理"多步资源申请"最常见的写法。
 */

/*
 * @file video_capture.c
 * @brief 实现从传感器到VI Channel的完整视频采集链路。
 *
 * 本模块负责MIPI设备、VI设备、VI Pipe和VI Channel的配置，
 * 并提供采集帧验证及NV21文件保存功能。
 */
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "ot_mipi_rx.h"
#include "video_capture.h"

#include "app_config.h"
#include "ss_mpi_vi.h"
#include "ss_mpi_sys_mem.h"

/* 设置 MIPI 接收器的 lane 分组方式。 */
/*
 * 【说明】第 0 步（实际在 start_sensor 之前被调用）：设置 MIPI RX 的 lane 分组。
 *   OT_MIPI_SET_HS_MODE 里的 HS = High Speed。LANE_DIVIDE_MODE_0 表示 4 条 lane 视为一个整体，
 *   我们的传感器只接了其中 2 条也没关系。
 */
td_s32 video_capture_set_mipi_lane_mode(void)
{
    int fd;
    td_s32 ret;

    /*
     * 模式0：四条数据 lane 资源归为一组，不要求全部启用。
     * 模式1：资源拆成两个组，每组两条数据 lane。
     */
    lane_divide_mode_t lane_mode = LANE_DIVIDE_MODE_0;

    // 打开 MIPI 接收器设备文件
    fd = open("/dev/ot_mipi_rx", O_RDWR);
    if (fd < 0) {
        perror("Failed to open /dev/ot_mipi_rx");
        return TD_FAILURE;
    }

    // 设置 MIPI 接收器的 lane 分组方式
    ret = ioctl(fd, OT_MIPI_SET_HS_MODE, &lane_mode);
    if (ret < 0) {
        perror("Failed to set MIPI RX lane mode");
        goto close_fd;
    }

    printf("MIPI lane mode configured\n");

close_fd:
    close(fd);
    return ret;
}

/* 使能 Sensor 时钟，并完成复位。 */
/* 补充：当前只准备时钟和复位条件，尚未写入 Sensor 模式寄存器。 */
/*
 * 【说明】第 1 步：让传感器"通电并开始工作"。
 *   传感器自己需要一个输入时钟(MCLK)才会运转，并且要经过"复位→释放复位"才进入正常状态。
 *   这两件事不是通过传感器的 I2C 寄存器做的，而是由 MIPI RX 驱动提供的 ioctl 完成的。
 *   顺序：开时钟 → 复位(拉低) → 释放复位(拉高)。
 */
td_s32 video_capture_start_sensor(void)
{
    int fd;
    td_s32 ret;

    /*
     * 【说明】sns_clk_source_t / sns_rst_source_t：SDK 里定义的"时钟源编号 / 复位源编号"类型。
     *   芯片上可能有多路时钟和多路复位引脚，0 表示第 0 路，即我们板子上接传感器的那一路。
     */
    sns_clk_source_t clk_source = 0; //选择传感器时钟源
    sns_rst_source_t rst_source = 0; //选择传感器复位源

    /*
     * 打开MIPI RX驱动的控制接口。
     * 该驱动同时提供Sensor时钟和复位控制；
     * open本身不代表已经启动MIPI图像接收。
     */
    fd = open("/dev/ot_mipi_rx", O_RDWR);
    if (fd < 0) {
        perror("Failed to open /dev/ot_mipi_rx");
        return TD_FAILURE;
    }

    //打开传感器时钟，通过clk_source选择具体的时钟源
    /*
     * 【说明】OT_MIPI_ENABLE_SENSOR_CLOCK：命令"打开传感器时钟"。
     *   传感器收不到时钟就像手机没通电，后面所有配置都没有意义。
     */
    ret = ioctl(fd, OT_MIPI_ENABLE_SENSOR_CLOCK, &clk_source);
    if (ret < 0) {
        perror("Failed to enable sensor clock");
        goto close_fd;
    }

    //复位传感器，通过rst_source选择具体的复位源
    ret = ioctl(fd, OT_MIPI_RESET_SENSOR, &rst_source);
    if (ret < 0) {
        perror("Failed to reset sensor");
        goto rollback_sensor;
    }

    //取消复位传感器
    ret = ioctl(fd, OT_MIPI_UNRESET_SENSOR, &rst_source);
    if (ret < 0) {
        perror("Failed to unreset sensor");
        goto rollback_sensor;
    }

    printf("Sensor clock enabled, reset released\n");

    /* 正常成功，跳过失败回滚，只关闭控制句柄。 */
    /*
     * 【说明】goto 的用法：跳到同一函数里下面某个标签处继续往下执行。
     *   这里跳到 close_fd，是因为"成功时也要 close(fd)"，但不应该走 rollback 的撤销代码。
     */
    goto close_fd;

/*
 * 【说明】回滚：走到这里说明"时钟已经开了，但后面某步失败了"，
 *   于是把传感器重新复位并关掉时钟，让硬件回到初始状态，避免留下半开状态。
 *   注意 ret 保持最初失败的返回值，不被清理动作覆盖。
 */
rollback_sensor:
    /*
     * 时钟已经开启，后续失败时尝试恢复停止状态。
     * 清理操作不覆盖 ret，保留最初失败操作的返回值。
     */
    if (ioctl(fd, OT_MIPI_RESET_SENSOR, &rst_source) < 0) {
        perror("Cleanup: failed to reset sensor");
    }

    if (ioctl(fd, OT_MIPI_DISABLE_SENSOR_CLOCK, &clk_source) < 0) {
        perror("Cleanup: failed to disable sensor clock");
    }

close_fd:
    close(fd);
    return ret;
}

/* 配置并启动 MIPI 接收器。 */
/*
 * 【说明】第 2 步：配置并启动 MIPI 接收器。
 *   MIPI CSI-2 是传感器和芯片之间传图像的高速串行协议。接收器必须"知道会收到什么"
 *   （原始 10bit、1 倍速、哪几条 lane、画面多大），才能把串行数据还原成像素。
 *   流程：开 MIPI 时钟 → 复位 → 写属性 → 写扩展数据类型 → 释放复位。
 */
td_s32 video_capture_start_mipi(void)
{
    int fd;
    td_s32 ret;

    /*选择MIPI接收器设备号*/
    combo_dev_t mipi_dev = 0;

    /*配置MIPI接收器属性,接收器接受mipi数据*/
    combo_dev_attr_t attr = {
        /*
         * 【说明】combo_dev_attr_t：MIPI 接收器的"属性表"（combo 是指一个物理口可兼容 MIPI/LVDS 等多种接口）。
         *   - devno      ：接收器编号 0
         *   - input_mode ：输入方式 = MIPI
         *   - data_rate  ：X1 = 每个时钟周期传 1 个像素（普通模式）
         *   - img_rect   ：要接收的画面区域，从(0,0)开始，宽高 2560x1440（与传感器输出一致）
         *   - input_data_type RAW_10BIT：传感器输出的是 10 位 RAW（未经 ISP 处理的原始拜耳阵列数据）
         *   - wdr_mode NONE：不使用宽动态(WDR)模式，只出一种曝光
         *   - lane_id {0,2,-1,-1}：传感器的第 0、1 条 lane 接到芯片的第 0、2 号 lane；-1 表示没接
         *                          （这个值由硬件原理图决定，接线不同就要改！）
         */
        .devno = mipi_dev,
        .input_mode = INPUT_MODE_MIPI,
        .data_rate = MIPI_DATA_RATE_X1,

        .img_rect = {
            .x = 0,
            .y = 0,
            .width = APP_VIDEO_WIDTH,
            .height = APP_VIDEO_HEIGHT
        },

        .mipi_attr = {
            .input_data_type = DATA_TYPE_RAW_10BIT,
            .wdr_mode = OT_MIPI_WDR_MODE_NONE,
            .lane_id = {0, 2, -1, -1} /* 传感器的lane0和lane1对应MIPI的lane 0、2 */
        }
    };

    /* 扩展数据类型结构体；采集前端：配置传感器接入、外设总线或控制时序，为RAW视频采集提供条件 */
    /*
     * 【说明】扩展数据类型：告诉 MIPI 接收器"除了常规图像包，还可能出现哪些数据类型的包"。
     *   0x2c 是 MIPI 协议里的数据类型号（RAW12），12 是位宽。
     *   这是 SDK 示例里针对该系列传感器的固定写法，初学阶段保持不动即可。
     */
    ext_data_type_t ext_attr = {
        .devno = 0,
        .num = 3,
        .ext_data_bit_width = {12, 12, 12},
        .ext_data_type = {0x2c, 0x2c, 0x2c}
    };

    fd = open("/dev/ot_mipi_rx", O_RDWR);
    if (fd < 0) {
        perror("Failed to open /dev/ot_mipi_rx");
        return TD_FAILURE;
    }

    /* 启用MIPI时钟 */
    /*
     * 【说明】注意"传感器时钟"和"MIPI 接收器时钟"是两回事：
     *   前者是给传感器芯片用的，后者是给我们 SoC 里的 MIPI 接收模块用的。
     */
    ret = ioctl(fd, OT_MIPI_ENABLE_MIPI_CLOCK, &mipi_dev);
    if (ret < 0) {
        perror("Failed to enable MIPI clock");
        goto close_fd;
    }

    /*设置MIPI设备进入复位状态*/
    ret = ioctl(fd, OT_MIPI_RESET_MIPI, &mipi_dev);
    if (ret < 0) {
        perror("Failed to reset MIPI device");
        goto rollback_mipi;
    }

    /* 提交接口、尺寸、数据类型、lane 映射等属性。 */
    ret = ioctl(fd, OT_MIPI_SET_DEV_ATTR, &attr);
    if (ret < 0) {
        perror("Failed to set MIPI device attributes");
        goto rollback_mipi;
    }

    /* 提交扩展数据类型属性。 */
    ret = ioctl(fd, OT_MIPI_SET_EXT_DATA_TYPE, &ext_attr);
    if (ret < 0) {
        perror("Failed to set MIPI extension data type");
        goto rollback_mipi;
    }

    /* 取消MIPI设备复位状态*/
    ret = ioctl(fd, OT_MIPI_UNRESET_MIPI, &mipi_dev);
    if (ret < 0) {
        perror("Failed to unreset MIPI device");
        goto rollback_mipi;
    }

    printf("MIPI started\n");

    /* 正常成功，跳过失败回滚，只关闭控制句柄。 */
    goto close_fd;

rollback_mipi:
    /*
     * MIPI 时钟已经开启，后续失败时尝试恢复停止状态。
     * 清理操作不覆盖 ret，保留最初失败操作的返回值。
     */
    if (ioctl(fd, OT_MIPI_RESET_MIPI, &mipi_dev) < 0) {
        perror("Cleanup: failed to reset MIPI");
    }

    if (ioctl(fd, OT_MIPI_DISABLE_MIPI_CLOCK, &mipi_dev) < 0) {
        perror("Cleanup: failed to disable MIPI clock");
    }

close_fd:
    close(fd);
    return ret;
}

/* 配置并启动 VI 设备。 */
/*
 * 【说明】第 3 步：配置并使能 VI Dev（视频输入设备）。
 *   VI Dev 是 MIPI 接收器后面的"接入口"，规定怎么解释进来的数据。
 *   这里的参数大多是"数字接口时序"设置（vsync/hsync 极性等），对 MIPI 接口基本都是固定值，
 *   直接沿用 SDK 示例即可，学习阶段不必逐个深究。
 *   关键的几个：
 *    - intf_mode MIPI ：输入接口是 MIPI
 *    - work_mode MULTIPLEX_1：不复用（只接 1 路）
 *    - data_type RAW   ：进来的是 RAW 数据
 *    - in_size         ：输入画面 2560x1440
 *    - ss_mpi_vi_set_dev_attr 设置属性 → ss_mpi_vi_enable_dev 使能
 */
td_s32 video_capture_start_vi_dev(void)
{
    td_s32 ret;
    ot_vi_dev vi_dev = 0;

    ot_vi_dev_attr dev_attr = {
        .intf_mode = OT_VI_INTF_MODE_MIPI,
        .work_mode = OT_VI_WORK_MODE_MULTIPLEX_1,

        .component_mask = {0xfff00000, 0x00000000},
        .scan_mode = OT_VI_SCAN_PROGRESSIVE,
        .ad_chn_id = {-1, -1, -1, -1},

        .data_seq = OT_VI_DATA_SEQ_YUYV,

        .sync_cfg = {
            .vsync = OT_VI_VSYNC_FIELD,
            .vsync_neg = OT_VI_VSYNC_NEG_HIGH,
            .hsync = OT_VI_HSYNC_VALID_SIG,
            .hsync_neg = OT_VI_HSYNC_NEG_HIGH,
            .vsync_valid = OT_VI_VSYNC_VALID_SIG,
            .vsync_valid_neg = OT_VI_VSYNC_VALID_NEG_HIGH,
            .timing_blank = {0}
        },

        .data_type = OT_VI_DATA_TYPE_RAW,
        .data_reverse = TD_FALSE,

        .in_size = {
            .width = APP_VIDEO_WIDTH,
            .height = APP_VIDEO_HEIGHT
        },

        .data_rate = OT_DATA_RATE_X1
    };

    /** 设置VI设备属性 */
    ret = ss_mpi_vi_set_dev_attr(vi_dev, &dev_attr);
    if (ret != TD_SUCCESS) {
        printf("Failed to set VI device attributes: %#x\n",
               (td_u32)ret);
        return ret;
    }

    ret = ss_mpi_vi_enable_dev(vi_dev);
    if (ret != TD_SUCCESS) {
        printf("Failed to enable VI device: %#x\n",
               (td_u32)ret);
        return ret;
    }

    printf("VI device started\n");
    return TD_SUCCESS;
}

/* 绑定 VI Dev 与 VI Pipe */
/*
 * 【说明】第 4 步：把 VI Dev 与 VI Pipe 绑定（Dev0 → Pipe0）。
 *   可以把 Dev 想成"进水口"，Pipe 想成"处理管线"；绑定后数据才会从 Dev 流进 Pipe。
 *   一个板子上可以有多个 Dev 和 Pipe，所以需要明确指定谁连谁。
 */
td_s32 video_capture_bind_pipe(void)
{
    td_s32 ret;

    ot_vi_dev vi_dev = 0;
    ot_vi_pipe vi_pipe = 0;

    ret = ss_mpi_vi_bind(vi_dev, vi_pipe);
    if (ret != TD_SUCCESS) {
        printf("Failed to bind VI device to VI pipe: %#x\n",
               (td_u32)ret);
        return ret;
    }

    printf("VI device bound to VI pipe successfully\n");
    return TD_SUCCESS;
}

/* 设置 VI Group 的WDR融合属性。 */
/*
 * 【说明】第 5 步：设置 VI Group 的 WDR 融合属性。
 *   WDR(宽动态)需要把"长曝光+短曝光"多张图融合成一张，融合用的几个 Pipe 被编成一个 Group。
 *   我们不用 WDR（wdr_mode = NONE），所以 Group 里只有 Pipe0，
 *   但 SDK 要求"即使不用 WDR 也要设置一下"，所以这一步是走流程。
 *   cache_line = 画面高度：WDR 融合时的行缓存数，不开 WDR 时按图像高度填即可。
 */
td_s32 video_capture_set_vi_group(void)
{
    td_s32 ret;
    ot_vi_grp group = 0;

    ot_vi_wdr_fusion_grp_attr grp_attr = {
        .wdr_mode=OT_WDR_MODE_NONE,
        .cache_line=APP_VIDEO_HEIGHT,
        .pipe_id={0},
        .pipe_reverse=TD_FALSE
    };

    ret = ss_mpi_vi_set_wdr_fusion_grp_attr(group, &grp_attr);
    if (ret != TD_SUCCESS) {
        printf("Failed to set VI group attributes: %#x\n",
               (td_u32)ret);
        return ret;
    }

    printf("VI group attributes set successfully\n");
    return TD_SUCCESS;

}

/* 创建、配置并启动 VI Pipe。 */
/*
 * 【说明】第 6 步：创建并启动 VI Pipe。
 *   三个动作必须按顺序：create(创建，分配资源) → set_online_clock(设处理速率) → start(启动)。
 *   Pipe 里的 isp_bypass = FALSE 表示数据会经过 ISP（图像信号处理器）：
 *   传感器出的 RAW 数据要经过 ISP 做去噪、白平衡、颜色校正等，才能变成好看的 YUV 图像。
 */
td_s32 video_capture_start_pipe(void)
{
    td_s32 ret;
    td_s32 cleanup_ret;
    ot_vi_pipe vi_pipe = 0;

    /*
     * 【说明】⚠ pixel_rate（每秒要处理的像素数）= 宽 x 高 x 帧率。
     *   这里把帧率 30 写死在代码里。以后如果你把传感器帧率改成 25 或 60，要同步修改这里，
     *   否则在线模式下 ISP 处理速率与实际不匹配，可能出现花屏或丢帧。
     *   app_config.h 里已经有 APP_VIDEO_FPS（isp_control.c 就在用它），
     *   建议把这里的 30 也换成 APP_VIDEO_FPS，让帧率只在一个地方修改。
     */
    td_u32 pixel_rate = APP_VIDEO_WIDTH * APP_VIDEO_HEIGHT * 30; // 假设帧率为30fps

    ot_vi_pipe_attr pipe_attr = {
        .pipe_bypass_mode = OT_VI_PIPE_BYPASS_NONE,
        .isp_bypass = TD_FALSE,
        .size = {
            .width = APP_VIDEO_WIDTH,
            .height = APP_VIDEO_HEIGHT
        },
        /*
         * 【说明】Pipe 的输入像素格式：RGB_BAYER_10BPP = 拜耳阵列、每像素 10 位，对应传感器的 RAW10。
         *   拜耳阵列：传感器每个像素只感一种颜色（R、G、B 之一），ISP 之后再"猜"出完整的 RGB。
         *   frame_rate_ctrl 的 -1 表示"不做帧率转换"，源是多少帧就输出多少帧。
         */
        .pixel_format =  OT_PIXEL_FORMAT_RGB_BAYER_10BPP,
        .compress_mode = OT_COMPRESS_MODE_NONE,

        .frame_rate_ctrl = {
            .src_frame_rate = -1,  // 源帧率未知，使用-1表示不限制
            .dst_frame_rate = -1      // 目标帧率未知，使用-1表示不限制
        }
    };

    /* 1. 根据属性创建Pipe，建立相应资源 */
    ret = ss_mpi_vi_create_pipe(vi_pipe, &pipe_attr);
    if (ret != TD_SUCCESS) {
        printf("Failed to create VI pipe: %#x\n",
               (td_u32)ret);
        return ret;
    }

    /* 2. 设置当前在线模式所需的像素处理速率 */
    /*
     * 【说明】在线模式(VI_ONLINE)下，VI 和后级通过片内总线直接对接，需要告诉硬件"每秒大概要处理多少像素"，
     *   它据此设置内部时钟频率。设低了会处理不过来，设太高则浪费功耗。
     */
    ret=ss_mpi_vi_set_pipe_online_clock(vi_pipe, pixel_rate);
    if (ret != TD_SUCCESS) {
        printf("Failed to set VI pipe online clock: %#x\n",
                (td_u32)ret);
            goto destroy_pipe;
        }

    ret = ss_mpi_vi_start_pipe(vi_pipe);
    if (ret != TD_SUCCESS) {
        printf("Failed to start VI pipe: %#x\n",
               (td_u32)ret);
        goto destroy_pipe;
    }

    printf("VI Pipe 0 created and started\n");
    return TD_SUCCESS;
/*
 * 【说明】回滚：Pipe 已创建成功但后续步骤失败，要把它销毁，防止资源泄漏。
 */
destroy_pipe:
    /*
     * Pipe已经创建，但后续配置或启动失败。
     * 销毁Pipe，并保留最初的失败返回值ret。
     */
    cleanup_ret = ss_mpi_vi_destroy_pipe(vi_pipe);
    if (cleanup_ret != TD_SUCCESS) {
        printf("Cleanup: failed to destroy VI Pipe: %#x\n",
               (td_u32)cleanup_ret);
    }

    return ret;
}

/* 配置并使能 VI Pipe 0 的 Channel 0。 */
/*
 * 【说明】第 7 步：配置并使能 VI Channel 0（管线的"出口"）。
 *   经过 ISP 处理后，图像从这里以 NV21 格式输出，下一级 VPSS 会来取。
 *   chn_attr 里重要的几项：
 *    - size                ：输出分辨率 2560x1440
 *    - pixel_format NV21   ：YVU 半平面 4:2:0（先 Y 整平面，再 VU 交错），每像素 1.5 字节
 *    - dynamic_range SDR8  ：标准动态范围、8 位
 *    - mirror_en / flip_en ：这里不翻转；翻转放在 ISP 里做（isp_control.c 里的 flip）
 *    - depth = 1           ：给用户程序保留 1 帧的"取帧队列"。
 *                            只有 depth>0，应用才能用 ss_mpi_vi_get_chn_frame 主动取帧
 *                            （test_frame / save_yuv 用到）。正式运行时数据通过 bind 自动
 *                            流给 VPSS，这个队列会额外占用一个缓存块，不用调试函数时可考虑改为 0。
 */
td_s32 video_capture_start_chn(void)
{
    td_s32 ret;

    ot_vi_pipe vi_pipe = 0;
    ot_vi_chn vi_chn = 0;

    /* 设置管线输出图像的属性 */
    ot_vi_chn_attr chn_attr = {
        .size = {
            .width = APP_VIDEO_WIDTH,
            .height = APP_VIDEO_HEIGHT
        },

        .pixel_format = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420,
        .dynamic_range = OT_DYNAMIC_RANGE_SDR8,
        .video_format = OT_VIDEO_FORMAT_LINEAR,
        .compress_mode = OT_COMPRESS_MODE_NONE,

        .mirror_en = TD_FALSE,
        .flip_en = TD_FALSE,

        /* 为后续应用主动取帧保留一帧队列 */
        .depth = 1,

        /* 本级不额外指定帧率转换 */
        .frame_rate_ctrl = {
            .src_frame_rate = -1,
            .dst_frame_rate = -1
        }
    };

    /* 1. 提交输出通道属性 */
    ret = ss_mpi_vi_set_chn_attr(vi_pipe, vi_chn, &chn_attr);
    if (ret != TD_SUCCESS) {
        printf("Failed to set VI Channel attributes: %#x\n",
               (td_u32)ret);
        return ret;
    }

    /* 2. 使能输出通道 */
    ret = ss_mpi_vi_enable_chn(vi_pipe, vi_chn);
    if (ret != TD_SUCCESS) {
        printf("Failed to enable VI Channel: %#x\n",
               (td_u32)ret);
        return ret;
    }

    printf("VI Pipe 0 Channel 0 enabled\n");
    return TD_SUCCESS;
}

/* 获取并释放一帧，用于验证VI通道是否输出图像 */
/*
 * 【说明】调试函数：验证 VI 通道是否真正在出图。
 *   ss_mpi_vi_get_chn_frame(pipe, chn, &frame_info, 超时毫秒)
 *       向通道"借"一帧。借来的是 SDK 内存池里的一块内存，不是拷贝。
 *   ss_mpi_vi_release_chn_frame(pipe, chn, &frame_info)
 *       用完必须"还"回去！不还的话内存池里的块越来越少，最后整条管线卡死。
 *   这种"借—还"配对，在后面的 VENC get_stream / release_stream 里还会再次出现。
 *   ot_video_frame_info：一帧图像的描述，里面的 video_frame 含宽高、stride(行跨度)、
 *   像素格式、时间戳 pts、物理地址 phys_addr 等。
 */
td_s32 video_capture_test_frame(void)
{
    td_s32 ret;
    ot_vi_pipe vi_pipe = 0;
    ot_vi_chn vi_chn = 0;

    /* 保存SDK返回的这一帧的描述信息 */
    ot_video_frame_info frame_info = {0};

    printf("Waiting for a frame from VI Pipe 0 Channel 0...\n");

    /* 最多等待3000毫秒 */
    ret = ss_mpi_vi_get_chn_frame(
        vi_pipe,
        vi_chn,
        &frame_info,
        3000);

    if (ret != TD_SUCCESS) {
        printf("Get VI frame failed: %#x\n", (td_u32)ret);
        return ret;
    }

    /* 获取成功后，才可以读取返回的帧信息 */
    printf("Get VI frame succeeded\n");

    printf("  size: %u x %u\n",
           frame_info.video_frame.width,
           frame_info.video_frame.height);

    /*
     * 【说明】stride（行跨度）：内存里"一行"实际占的字节数，可能大于图像宽度（有对齐填充）。
     *   例如宽度 2560，stride 可能是 2560 或 2576。所以读图像时必须按 stride 跳行，不能按宽度。
     */
    printf("  stride: Y=%u, VU=%u\n",
           frame_info.video_frame.stride[0],
           frame_info.video_frame.stride[1]);

    printf("  pixel format: %d\n",
           (int)frame_info.video_frame.pixel_format);

    printf("  time_ref: %u, pts: %llu\n",
           frame_info.video_frame.time_ref,
           (unsigned long long)frame_info.video_frame.pts);

    /*
     * 获取成功后必须释放。
     * 把这一帧交还给SDK，允许其缓冲区后续被重复使用。
     */
    ret = ss_mpi_vi_release_chn_frame(
        vi_pipe,
        vi_chn,
        &frame_info);

    if (ret != TD_SUCCESS) {
        printf("Release VI frame failed: %#x\n", (td_u32)ret);
        return ret;
    }

    printf("VI frame released\n");
    return TD_SUCCESS;
}

/* 从VI通道获取一帧，保存为无压缩的NV21文件 */
/*
 * 【说明】调试函数：从 VI 通道取一帧，写成 .yuv 文件，可以用 YUV 查看器打开检查画面。
 *   难点在于"物理地址不能直接用"：
 *     SDK 给的 phys_addr 是物理内存地址，Linux 用户程序只能访问"虚拟地址"，
 *     所以要用 ss_mpi_sys_mmap(物理地址, 长度) 映射成虚拟地址才能读写，用完 munmap 解除。
 *   整体流程：取帧 → 检查格式 → 算长度 → mmap → fopen → 按行写 Y 平面 → 按行写 VU 平面
 *            → cleanup(关文件、munmap、还帧)
 *   cleanup 标签：不管成功失败，最后都要走这里把"借的帧、映射的内存、打开的文件"全部还掉。
 */
td_s32 video_capture_save_yuv(const char *file_path)
{
    td_s32 ret;
    td_s32 cleanup_ret;
    ot_vi_pipe vi_pipe = 0;
    ot_vi_chn vi_chn = 0;

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
    ret = ss_mpi_vi_get_chn_frame(
        vi_pipe, vi_chn, &frame_info, 3000);
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
    /*
     * 【说明】NV21 有两个"平面"（plane）：
     *   Y 平面  ：height 行，每行 stride[0] 字节（亮度，黑白图像本身）
     *   VU 平面 ：height/2 行，每行 stride[1] 字节（颜色信息，V 和 U 交错存放，分辨率是 Y 的一半）
     *   所以一帧大小 ≈ width*height*1.5。
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
    /*
     * 【说明】逐行写：每行只写 width 个有效字节，把行末的对齐填充丢掉，
     *   这样文件才是"紧凑"的标准 NV21，查看器才能正确显示。
     *   fwrite(指针, 每个元素字节数, 元素个数, 文件)：返回实际写出的元素个数，不等于期望值就是写失败。
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
    cleanup_ret = ss_mpi_vi_release_chn_frame(
        vi_pipe, vi_chn, &frame_info);
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

/* 停止 VI Pipe 0 的 Channel 0 输出。 */
/*
 * 【说明】禁用 VI Channel 0。
 */
void video_capture_stop_chn(void)
{
    td_s32 ret;

    ot_vi_pipe vi_pipe = 0;
    ot_vi_chn vi_chn = 0;

    ret = ss_mpi_vi_disable_chn(vi_pipe, vi_chn);
    if (ret != TD_SUCCESS) {
        printf("Failed to disable VI Channel: %#x\n",
               (td_u32)ret);
        return;
    }

    printf("VI Pipe 0 Channel 0 disabled\n");
}

/* 停止并销毁 VI Pipe。 */
/*
 * 【说明】停止顺序：先 stop（停止运行）再 destroy（销毁释放）。
 *   即使 stop 失败也继续 destroy，因为我们的目标是"尽量把资源还回去"。
 */
void video_capture_stop_pipe(void)
{
    td_s32 stop_ret;
    td_s32 destroy_ret;
    ot_vi_pipe vi_pipe = 0;

    /* 1. 停止管线运行 */
    stop_ret = ss_mpi_vi_stop_pipe(vi_pipe);
    if (stop_ret != TD_SUCCESS) {
        printf("Failed to stop VI Pipe: %#x\n",
               (td_u32)stop_ret);
    }

    /* 2. 即使停止报错，也继续尝试销毁并打印结果 */
    destroy_ret = ss_mpi_vi_destroy_pipe(vi_pipe);
    if (destroy_ret != TD_SUCCESS) {
        printf("Failed to destroy VI Pipe: %#x\n",
               (td_u32)destroy_ret);
    }

    if (stop_ret == TD_SUCCESS &&
        destroy_ret == TD_SUCCESS) {
        printf("VI Pipe 0 stopped and destroyed\n");
    }
}

/* 解除 VI Dev 与 VI Pipe 的绑定。 */
void video_capture_unbind_pipe(void)
{
    td_s32 ret;

    ot_vi_dev vi_dev = 0;
    ot_vi_pipe vi_pipe = 0;

    ret = ss_mpi_vi_unbind(vi_dev, vi_pipe);
    if (ret != TD_SUCCESS) {
        printf("Failed to unbind VI device from VI pipe: %#x\n",
               (td_u32)ret);
        return;
    }

    printf("VI device unbound from VI pipe successfully\n");
}

/* 停止 VI 设备。 */
/*
 * 【说明】停止 VI Dev：对应 ss_mpi_vi_disable_dev。注意这里函数返回 void，失败只打印不再向上报告。
 */
void video_capture_stop_vi_dev(void)
{
    td_s32 ret;
    ot_vi_dev vi_dev = 0;

    ret = ss_mpi_vi_disable_dev(vi_dev);
    if (ret != TD_SUCCESS) {
        printf("Failed to disable VI device: %#x\n",
               (td_u32)ret);
        return;
    }

    printf("VI device stopped\n");
}

/* 复位 MIPI 接收器并关闭接收器时钟。 */
/*
 * 【说明】停止 MIPI：先复位再关时钟。两步即使其中一步失败也要继续做下一步（尽力清理），
 *   用 stop_ok 记录是否全部成功，成功才打印 "MIPI stopped"。
 */
void video_capture_stop_mipi(void)
{
    int fd;
    td_s32 ret;
    td_bool stop_ok = TD_TRUE;

    combo_dev_t mipi_dev = 0;

    fd = open("/dev/ot_mipi_rx", O_RDWR);
    if (fd < 0) {
        perror("Failed to open /dev/ot_mipi_rx");
        return;
    }

    /* 设置MIPI设备进入复位状态 */
    ret = ioctl(fd, OT_MIPI_RESET_MIPI, &mipi_dev);
    if (ret < 0) {
        perror("Failed to reset MIPI device");
        stop_ok = TD_FALSE;
        /* 不提前返回，继续尝试关闭接收器时钟。 */
    }

    /* 禁用MIPI时钟 */
    ret = ioctl(fd, OT_MIPI_DISABLE_MIPI_CLOCK, &mipi_dev);
    if (ret < 0) {
        perror("Failed to disable MIPI clock");
        stop_ok = TD_FALSE;
    }

    close(fd);

    /* 只有两步均成功，才打印停止成功。 */
    if (stop_ok == TD_TRUE) {
        printf("MIPI stopped\n");
    }

    return;
}

/* 停止传感器采集并关闭时钟。 */
/*
 * 【说明】停止传感器：复位 + 关时钟，正好是 start_sensor 的反操作。
 *   这是 main.c 清理链里最后一个硬件步骤（最先启动的，最后关闭）。
 */
void video_capture_stop_sensor(void)
{
    int fd;
    td_s32 ret;

    sns_clk_source_t clk_source = 0; //选择传感器时钟源
    sns_rst_source_t rst_source = 0; //选择传感器复位源

    //先打开 MIPI 接收器设备文件
    fd = open("/dev/ot_mipi_rx", O_RDWR);
    if (fd < 0) {
        perror("Failed to open /dev/ot_mipi_rx");
        return;
    }

    //设置传感器为复位状态
    ret = ioctl(fd, OT_MIPI_RESET_SENSOR, &rst_source);
    if (ret < 0) {
        perror("Failed to reset sensor");
        /* 不提前返回，继续尝试关闭传感器时钟。 */
    }

    //关闭传感器时钟
    ret = ioctl(fd, OT_MIPI_DISABLE_SENSOR_CLOCK, &clk_source);
    if (ret < 0) {
        perror("Failed to disable sensor clock");
    }

    close(fd);
}
