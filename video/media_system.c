/*
 * 【说明】本文件做两件事：
 *   1) media_system_init()   ：划分视频内存池(VB) → 初始化 VB/SYS 模块 → 设置 VI 与 VPSS 的工作模式
 *   2) media_system_deinit() ：退出时按相反顺序释放（先 SYS 再 VB）
 *
 *   【术语】
 *   - VB  (Video Buffer)  ：视频缓存池。SDK 在开机时一次性申请好一批"大块内存"，
 *                           各硬件模块之间传图像时，不拷贝数据，只传"这块内存的编号"。
 *   - SYS               ：系统控制模块，负责模块之间的绑定(bind)等。
 *   - ss_mpi_xxx        ：海思闭源库提供的函数（MPI = Media Process Interface，媒体处理接口）。
 *                           我们看不到它们的源码，只能按文档调用。
 */

#include <stdio.h>
/*
 * @file media_system.c
 * @brief 配置公共视频缓存池并初始化媒体系统。
 */
#include "app_config.h"
#include "media_system.h"

#include "ot_buffer.h"
#include "ss_mpi_vb.h"
#include "ss_mpi_sys.h"
/* 根据输入和VPSS输出规格建立公共视频缓存池。 */
/*
 * 【说明】ot_xxx 开头的类型都是 SDK 头文件里定义的结构体/枚举。
 *   "= {0}" 的意思是：把整个结构体清零，避免里面有随机垃圾值（C 语言局部变量默认不清零）。
 */
td_s32 media_system_init(void)
{
    td_s32 ret;
    td_u32 block_size;
    ot_vi_vpss_mode work_mode = {0};
    td_u32 i;
    ot_pic_buf_attr buf_attr={0};

    ot_vb_cfg vb_cfg = {0};

    /* 描述需要存放的一张图像。 */
    buf_attr.width = APP_VIDEO_WIDTH;
    buf_attr.height = APP_VIDEO_HEIGHT;
    /*
     * 【说明】先"描述"要存的一张图：宽、高、对齐、位宽、像素格式……
     *   align = 16：每一行的字节数要对齐到 16 的倍数（硬件读写内存时按块对齐会更快、才不会出错）。
     *   OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420 就是我们常说的 NV21：
     *     先放整张 Y(亮度)平面，再放 V、U 交错的色度平面，每个像素平均 1.5 字节。
     *   compress_mode NONE：不压缩；video_format LINEAR：线性排列（普通的一行接一行）。
     */
    buf_attr.align = 16;
    buf_attr.bit_width = OT_DATA_BIT_WIDTH_8;
    buf_attr.pixel_format = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    buf_attr.compress_mode = OT_COMPRESS_MODE_NONE;
    buf_attr.video_format = OT_VIDEO_FORMAT_LINEAR;

    /* 计算一张输入图像所需的缓存块大小。 */
    /*
     * 【说明】ot_common_get_pic_buf_size：SDK 提供的"算尺寸"函数。
     *   输入"一张图的描述"，返回"存下这张图需要多少字节"。
     *   2560x1440 的 NV21 大约 5.5 MB；返回 0 表示参数不合法。
     */
    block_size=ot_common_get_pic_buf_size(&buf_attr);
    if (block_size == 0) {
        printf("get picture buffer size failed\n");
        return -1;
    }

    /* 配置输入图像缓存池，每个块容纳一张完整图像。 */
    /*
     * 【说明】配置内存池：
     *   - max_pool_cnt：最多允许存在多少个池（含后面各模块自己私有创建的池），128 是留足余量。
     *   - common_pool[0]：第 0 号"公共池"，给 VI 出来的 2560x1440 原图用，共 4 块，每块 blk_size 字节。
     *   - remap_mode NONE：不需要把这块内存映射给用户态程序读写（我们只在 save_yuv 时临时映射）。
     *   块数 blk_cnt=4 是"同时有多少张图在流水线里"；太少会因为没有空闲块而丢帧。
     */
    vb_cfg.max_pool_cnt = 128;
    vb_cfg.common_pool[0].blk_size = block_size;
    vb_cfg.common_pool[0].blk_cnt = 4;
    vb_cfg.common_pool[0].remap_mode = OT_VB_REMAP_MODE_NONE;

    /* 为VPSS输出的1280×720图像准备公共缓存 */
    /*
     * 【说明】再算一个池：VPSS 缩放后输出的 1280x720 图像，供 VENC 编码使用。
     *   同一个 buf_attr 只改宽高就能复用。
     */
    buf_attr.width = APP_VPSS_OUT_WIDTH;
    buf_attr.height = APP_VPSS_OUT_HEIGHT;
    block_size = ot_common_get_pic_buf_size(&buf_attr);
    if (block_size == 0) {
        printf("get picture buffer size failed\n");
        return -1;
    }

    vb_cfg.common_pool[1].blk_size = block_size;
    vb_cfg.common_pool[1].blk_cnt = 4;
    vb_cfg.common_pool[1].remap_mode = OT_VB_REMAP_MODE_NONE;


    /*
     * 为 VPSS 通道 2（DMS 驾驶员监控用的 640x640）准备单独的缓存池。
     * 单独建池：避免和编码器抢 pool1 的块，也避免浪费大块内存。
     * 块数 3：VPSS 正在写 1 块 + 队列里等取 1 块 + DMS 线程手上处理 1 块。
     */
    buf_attr.width = APP_DMS_IN_WIDTH;
    buf_attr.height = APP_DMS_IN_HEIGHT;
    block_size = ot_common_get_pic_buf_size(&buf_attr);
    if (block_size == 0) {
        printf("get picture buffer size failed\n");
        return -1;
    }

    vb_cfg.common_pool[2].blk_size = block_size;
    vb_cfg.common_pool[2].blk_cnt = 3;
    vb_cfg.common_pool[2].remap_mode = OT_VB_REMAP_MODE_NONE;

    /*
     * 【说明】三步初始化，顺序不能乱：
     *   ss_mpi_vb_set_cfg  ：把上面写好的"池配置"告诉 SDK（只是登记，还没真正分配内存）
     *   ss_mpi_vb_init     ：真正去申请内存
     *   ss_mpi_sys_init    ：启动 SYS 模块（它要用到 VB 内存池，所以必须在 VB 之后）
     *   某一步失败时，要把前面已经成功的步骤"撤销"（比如 sys_init 失败要调 vb_exit），
     *   这就是后面 goto 清理写法的基本思想：谁先成功，谁后撤销。
     */
    ret = ss_mpi_vb_set_cfg(&vb_cfg);
    if (ret != 0) {
        printf("set vb config failed\n");
        return ret;
    }

    ret = ss_mpi_vb_init();
    if (ret != 0) {
        printf("vb init failed\n");
        return ret;
    }

    ret = ss_mpi_sys_init();
    if (ret != 0) {
        printf("sys init failed\n");
        ss_mpi_vb_exit();
        return ret;
    }

    printf("SYS/VB initialized: pool0=%llu bytes x %u, "
       "pool1=%llu bytes x %u, "
       "pool2=%llu bytes x %u\n",
       (unsigned long long)vb_cfg.common_pool[0].blk_size,
       vb_cfg.common_pool[0].blk_cnt,
       (unsigned long long)vb_cfg.common_pool[1].blk_size,
       vb_cfg.common_pool[1].blk_cnt,
       (unsigned long long)vb_cfg.common_pool[2].blk_size,
       vb_cfg.common_pool[2].blk_cnt);

    /* 先将所有VI管线设置为安全的离线工作模式。 */
    /*
     * 【说明】设置"VI 与 VPSS 的工作模式"：
     *   - ONLINE ：VI 采到的数据不落到 DDR 内存，直接通过芯片内部通路送给下一级（省内存带宽，延迟低）
     *   - OFFLINE：VI 先把整帧写进内存，下一级再从内存读（更灵活，但多占带宽）
     *   这里先把所有 Pipe 设成最保险的 OFFLINE，再把 Pipe0 改成"VI 在线、VPSS 离线"。
     *   （我们只用了 Pipe0，这是 SDK 示例里针对该板子的推荐组合。）
     */
    for (i = 0; i < OT_VI_MAX_PHYS_PIPE_NUM; i++) {
        work_mode.mode[i] = OT_VI_OFFLINE_VPSS_OFFLINE;
    }

    /* 管线0采用VI在线、VPSS离线模式，匹配当前处理链路。 */
    work_mode.mode[0] = OT_VI_ONLINE_VPSS_OFFLINE;

    ret=ss_mpi_sys_set_vi_vpss_mode(&work_mode);
    if (ret != 0) {
        printf("set vi-vpss mode failed\n");
        ss_mpi_sys_exit();
        ss_mpi_vb_exit();
        return ret;
    }

    return 0;
}

/* 退出系统模块并释放视频缓存池。 */
/*
 * 【说明】退出：与初始化的顺序相反（后初始化的先退出）。
 *   这里即使 sys_exit 失败也继续调用 vb_exit，因为"尽量多释放一点"总比半途退出好。
 *   %#x 的意思是用十六进制打印并带 0x 前缀，SDK 的错误码是 0x80xxxxxx 这样的形式，十六进制更好查文档。
 */
void media_system_deinit(void)
{
    td_s32 ret;

    /* 先退出媒体系统，再释放底层视频缓存。 */
    ret = ss_mpi_sys_exit();
    if (ret != TD_SUCCESS) {
        printf("SYS exit failed: %#x\n", (td_u32)ret);
    }

    ret = ss_mpi_vb_exit();
    if (ret != TD_SUCCESS) {
        printf("VB exit failed: %#x\n", (td_u32)ret);
    }
}
