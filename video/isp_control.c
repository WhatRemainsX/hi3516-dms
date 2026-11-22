/*
 * 【说明】本文件的核心流程（顺序不能乱）：
 *   isp_control_register()  登记传感器驱动 + 3A 算法      （不碰硬件，只是"注册"）
 *   isp_control_init()      设置参数 → 分配 ISP 内存 → 设公共属性 → ss_mpi_isp_init
 *                           （此时 SDK 会通过 I2C 去配置传感器的模式寄存器，传感器才真正开始出图）
 *   isp_control_start()     创建线程，运行 ss_mpi_isp_run()
 *   isp_control_deinit()    ss_mpi_isp_exit 让 run 返回 → pthread_join 等线程结束
 *   isp_control_unregister() 注销 AWB → AE → 传感器回调（注册的反序）
 *
 *   【特别重要的一条规则】先让 ISP 线程完全结束(join)，才能注销算法、释放内存；
 *   否则线程还在用这些东西，会崩溃。deinit 里就是这么做的。
 */

/*
 * @file isp_control.c
 * @brief 实现Sensor回调、AE/AWB算法注册及ISP工作线程管理。
 *
 * ISP初始化和退出必须遵循资源依赖顺序，工作线程退出后
 * 才能注销算法和释放ISP内存。
 */
#include <stdio.h>

#include "isp_control.h"
#include "ot_sns_ctrl.h"
#include "ss_mpi_ae.h"
#include "ss_mpi_awb.h"
#include "app_config.h"
#include "ss_mpi_isp.h"
#include <pthread.h>
#include <string.h>

/*
 * SC4336P驱动对象。
 * 对象声明来自ot_sns_ctrl.h，
 * 实现来自SDK的libsns_sc4336p.a。
 */
/*
 * 【说明】g_ 开头：全局变量(global)；static：只在本 .c 文件内可见（别的文件看不到，避免重名）。
 *   g_sns_sc4336p_obj 是海思预编译库 libsns_sc4336p.a 里提供的"传感器对象"，
 *   里面是一堆函数指针(pfn_ = pointer to function)：注册回调、设置总线、设置模式……
 *   它就是"SC4336P 传感器的驱动适配层"，换传感器型号时，通常只需要换这个对象。
 */
static ot_isp_sns_obj *g_sensor = &g_sns_sc4336p_obj;

/* 自动曝光算法标识，当前对应Pipe 0 */
/*
 * 【说明】3A 算法库描述：
 *   AE  = Auto Exposure，自动曝光：根据画面亮度自动调快门时间、增益，让画面不过暗/过曝。
 *   AWB = Auto White Balance，自动白平衡：根据光源色温校正颜色，让白色看起来是白的。
 *   .id = 0 对应 Pipe 0；lib_name 是库的名字字符串，注册/注销时靠它认出是哪一套算法。
 */
static ot_isp_3a_alg_lib g_ae_lib = {
    .id = 0,
    .lib_name = OT_AE_LIB_NAME
};

/* 自动白平衡算法标识，当前对应Pipe 0 */
static ot_isp_3a_alg_lib g_awb_lib = {
    .id = 0,
    .lib_name = OT_AWB_LIB_NAME
};

/* 记录ISP内存是否已经初始化，供失败回滚和正常退出使用 */
/*
 * 【说明】三个"状态标志"，用来保证清理时"只清理已经成功建立过的东西"：
 *   g_isp_mem_ready       ISP 内存已经 mem_init 成功
 *   g_isp_initialized     ss_mpi_isp_init 已经成功
 *   g_isp_thread_created  工作线程已经创建（只有创建了才能 join，否则 join 一个不存在的线程会出错）
 *   td_bool 是海思定义的布尔类型：TD_TRUE / TD_FALSE。
 */
static td_bool g_isp_mem_ready = TD_FALSE;

/* 记录ss_mpi_isp_init是否已经成功 */
static td_bool g_isp_initialized = TD_FALSE;

/* 当前程序使用Pipe 0，其关联的ISP也通过编号0访问 */
static ot_vi_pipe g_isp_pipe = 0;

/* 保存ISP工作线程的标识，退出时用于等待线程结束 */
static pthread_t g_isp_thread;

/* 记录线程是否创建成功，只有创建成功才能等待它结束 */
static td_bool g_isp_thread_created = TD_FALSE;

/* 注册Sensor回调、指定控制总线，并注册AE/AWB算法 */
/* 注册Sensor适配回调及AE/AWB算法库。 */
/*
 * 【说明】注册分四步，任何一步失败都要把前面已成功的撤销（unregister_ae / unregister_sensor 标签）：
 *    1. pfn_register_callback ：把 SC4336P 的"曝光/增益/模式切换"函数交给 ISP 框架
 *    2. pfn_set_bus_info      ：告诉驱动：传感器挂在 I2C 总线 0 上
 *                               （I2C 是两根线的低速总线，用来读写传感器内部寄存器。
 *                                注意这跟 MIPI 不同：MIPI 传图像，I2C 传"控制命令"）
 *    3. ss_mpi_ae_register    ：把 AE 库登记到 ISP
 *    4. ss_mpi_awb_register   ：把 AWB 库登记到 ISP
 */
td_s32 isp_control_register(void)
{
    td_s32 ret;
    td_s32 cleanup_ret;
    ot_vi_pipe vi_pipe = 0;

    /* 沿用当前项目的Sensor控制总线：I2C 0 */
    ot_isp_sns_commbus bus_info = {
        .i2c_dev = 0
    };

    /* 检查当前Sensor对象是否提供了需要的接口 */
    if (g_sensor->pfn_register_callback == TD_NULL ||
        g_sensor->pfn_un_register_callback == TD_NULL ||
        g_sensor->pfn_set_bus_info == TD_NULL) {
        printf("Required Sensor callbacks are missing\n");
        return TD_FAILURE;
    }

    /* 1. 向驱动注册SC4336P的Sensor控制回调 ，将来 ISP 使用 AE、AWB，能够找到 SC4336P 的适配方法。*/
    ret = g_sensor->pfn_register_callback(
        vi_pipe, &g_ae_lib, &g_awb_lib);
    if (ret != TD_SUCCESS) {
        printf("Register Sensor callbacks failed: %#x\n",
               (td_u32)ret);
        return ret;
    }

    /* 2. 指定后续访问Sensor寄存器时使用的I2C总线 */
    ret = g_sensor->pfn_set_bus_info(vi_pipe, bus_info);
    if (ret != TD_SUCCESS) {
        printf("Set Sensor bus information failed: %#x\n",
               (td_u32)ret);
        goto unregister_sensor;
    }

    /* 3. 向ISP框架注册自动曝光算法 ，供ISP使用。 */
    ret = ss_mpi_ae_register(vi_pipe, &g_ae_lib);
    if (ret != TD_SUCCESS) {
        printf("Register AE failed: %#x\n",
               (td_u32)ret);
        goto unregister_sensor;
    }

    /* 4. 向ISP框架注册自动白平衡算法 ，供ISP使用。 */
    ret = ss_mpi_awb_register(vi_pipe, &g_awb_lib);
    if (ret != TD_SUCCESS) {
        printf("Register AWB failed: %#x\n",
               (td_u32)ret);
        goto unregister_ae;
    }

    printf("Sensor callbacks, I2C bus, AE and AWB registered\n");
    return TD_SUCCESS;

unregister_ae:
    /* AE已经注册，AWB注册失败时先回收AE */
    cleanup_ret = ss_mpi_ae_unregister(vi_pipe, &g_ae_lib);
    if (cleanup_ret != TD_SUCCESS) {
        printf("Cleanup: unregister AE failed: %#x\n",
               (td_u32)cleanup_ret);
    }

unregister_sensor:
    /* 回收此前注册成功的Sensor回调 */
    cleanup_ret = g_sensor->pfn_un_register_callback(
        vi_pipe, &g_ae_lib, &g_awb_lib);
    if (cleanup_ret != TD_SUCCESS) {
        printf("Cleanup: unregister Sensor failed: %#x\n",
               (td_u32)cleanup_ret);
    }

    /* 保留最初失败操作的返回值 */
    return ret;
}

/* 初始化ISP内存、公共属性及Sensor模式 */
/* 初始化ISP内存、控制参数、公共属性及Sensor模式。 */
/*
 * 【说明】ISP 初始化，分 5 步：get_ctrl_param/set_ctrl_param → mem_init → set_pub_attr → isp_init。
 *   ctrl_param 是 ISP "怎么运行"的参数；pub_attr（public attribute，公共属性）是
 *   "图像本身是什么样"的描述：输出窗口、传感器尺寸、帧率、拜耳排列、是否翻转……
 */
td_s32 isp_control_init(void)
{
    td_s32 ret;
    td_s32 cleanup_ret;
    ot_vi_pipe vi_pipe = 0;

    ot_isp_ctrl_param ctrl_param = {0};

    /*设置isp的公共属性*/
    ot_isp_pub_attr pub_attr= {
        .wnd_rect = {
            .x = 0,
            .y = 0,
            .width = APP_VIDEO_WIDTH,
            .height = APP_VIDEO_HEIGHT
        },
        .sns_size = {
            .width = APP_VIDEO_WIDTH,
            .height = APP_VIDEO_HEIGHT
        },
        /*
         * 【说明】关键属性解释：
         *   - wnd_rect / sns_size ：ISP 处理的窗口和传感器输出尺寸（都是 2560x1440，不裁剪）
         *   - frame_rate          ：传感器帧率，ISP 的曝光计算要用它（曝光时间不能超过一帧时间）
         *   - bayer_format BGGR   ：拜耳阵列的排列顺序（2x2 像素块里是 B G / G R）。
         *                           排错了会导致颜色偏紫、偏绿，必须与传感器手册一致。
         *   - sns_flip_en = TRUE  ：开启传感器垂直翻转(flip)。这应该是因为镜头/板子装得是倒的，
         *                           在传感器里直接翻转，比之后软件翻转省资源。画面倒了就改这里。
         *   - sns_mirror_en       ：水平镜像，目前关闭
         *   - mipi_crop            ：MIPI 裁剪功能，未启用
         */
        .frame_rate = (td_float)APP_VIDEO_FPS,
        .bayer_format = OT_ISP_BAYER_BGGR,
        .wdr_mode = OT_WDR_MODE_NONE,
        .sns_mode = 0,
        .sns_flip_en = TD_TRUE,
        .sns_mirror_en = TD_FALSE,

        .mipi_crop_attr = {
            .mipi_crop_en = TD_FALSE,
            .mipi_crop_offset = {
                .x = 0,
                .y = 0,
                .width = APP_VIDEO_WIDTH,
                .height = APP_VIDEO_HEIGHT
            }
        }
    };

    /*
     * 【说明】防重入：如果已经初始化过，不要重复初始化（会重复分配内存）。
     *   ⚠ 注意：这里返回的是 TD_SUCCESS（相当于"已经好了，不用做了"），
     *   如果真的发生这种情况，调用者不会知道其实什么都没做，调试时请留意这点。
     */
    if (g_isp_mem_ready || g_isp_thread_created) {
        printf("ISP resources already exist; deinitialize first\n");
        return TD_SUCCESS;
    }

     /* 1. 先读取现有控制参数，再修改需要的字段 */
    ret = ss_mpi_isp_get_ctrl_param(vi_pipe, &ctrl_param);
    if (ret != TD_SUCCESS) {
        printf("Get ISP control parameters failed: %#x\n",
               (td_u32)ret);
        return ret;
    }
    /* 沿用原样例：前端帧开始唤醒，关闭快速启动 */
    /*
     * 【说明】isp_run_wakeup_select：ss_mpi_isp_run 这个阻塞循环"被什么事件唤醒"。
     *   FE_START = Front End Start，前端每收到新一帧的开始就唤醒一次，
     *   这样 ISP 线程就能每帧运行一次 3A 算法。
     *   quick_start_en = FALSE：关闭"快速启动"功能（那是为了开机秒出图的专用优化，我们不用）。
     */
    ctrl_param.isp_run_wakeup_select = OT_ISP_RUN_WAKEUP_FE_START;
    ctrl_param.quick_start_en = TD_FALSE;

    /* 2. 设置ISP控制参数 */
    ret = ss_mpi_isp_set_ctrl_param(vi_pipe, &ctrl_param);
    if (ret != TD_SUCCESS) {
        printf("Set ISP control parameters failed: %#x\n",
               (td_u32)ret);
        return ret;
    }

    /* 3. 初始化ISP内存 */
    /*
     * 【说明】ss_mpi_isp_mem_init：给 ISP 分配运行时需要的内存（保存各种统计数据、LUT 表等）。
     *   成功后设置 g_isp_mem_ready，之后任何失败都要调用 deinit 来释放它。
     */
    ret = ss_mpi_isp_mem_init(vi_pipe);
    if (ret != TD_SUCCESS) {
        printf("ISP memory initialization failed: %#x\n",
               (td_u32)ret);
        return ret;
    }
    g_isp_mem_ready = TD_TRUE;

    /* 4. 设置ISP公共属性 */
    ret = ss_mpi_isp_set_pub_attr(vi_pipe, &pub_attr);
    if (ret != TD_SUCCESS) {
        printf("Set ISP public attributes failed: %#x\n",
               (td_u32)ret);
        goto cleanup_isp;
    }

    /*初始化ISP，SDK在这个阶段调用已注册的Sensor适配函数，触发Sensor模式寄存器初始化。*/
    /*
     * 【说明】ss_mpi_isp_init：真正让 ISP 开始工作。
     *   在这个函数内部，SDK 会回调前面注册的传感器函数，通过 I2C 把 SC4336P
     *   配置成 2560x1440@30fps 的工作模式（写一大堆模式寄存器）——传感器从这一刻才真的"出图"。
     *   这就是为什么 sns 的 clock/reset 是在 video_capture 里先做，而寄存器配置要等到这里。
     */
    ret= ss_mpi_isp_init(vi_pipe);
    if (ret != TD_SUCCESS) {
        printf("ISP initialization failed: %#x\n",
               (td_u32)ret);
        goto cleanup_isp;
    }

    g_isp_initialized = TD_TRUE;

    printf("ISP memory initialized, public attributes set, and ISP started\n");
    return TD_SUCCESS;

/*
 * 【说明】失败回滚：mem_init 已成功但后面失败，调用 isp_control_deinit 把内存还回去。
 */
cleanup_isp:
    /* 后续失败，回收已建立的ISP资源 */
    cleanup_ret = isp_control_deinit();
    if (cleanup_ret != TD_SUCCESS) {
        printf("Cleanup: ISP deinitialization failed: %#x\n",
               (td_u32)cleanup_ret);
    }
    return ret;

}

/* ISP线程入口：调用SDK运行接口处理持续到来的ISP事件。 */
/*
 * 【说明】线程入口函数。pthread 规定线程函数必须长这样：  void *函数名(void *参数)
 *   参数 arg 这里传的是 &g_isp_pipe（pipe 编号 0）的地址，函数里强转回 ot_vi_pipe* 再取值。
 *   ss_mpi_isp_run(pipe)：ISP 的主循环，会一直阻塞，每来一帧就处理一次 ISP 事件，
 *                         运行 AE/AWB 并更新传感器曝光。直到 ss_mpi_isp_exit 被调用才返回。
 */
static void *isp_control_thread(void *arg)
{
    td_s32 ret;
    ot_vi_pipe vi_pipe = *(ot_vi_pipe *)arg;
    printf("ISP thread entering run loop, pipe=%d\n", vi_pipe);
    /* 线程入口，等待ISP中断事件 */
    ret = ss_mpi_isp_run(vi_pipe);
    if (ret != TD_SUCCESS) {
        printf("ISP thread run failed: %#x\n", (td_u32)ret);
    }

    printf("ISP thread exiting run loop, pipe=%d\n", vi_pipe);
    return TD_NULL;
}

/* 创建ISP工作线程；调用前必须完成ISP初始化。 */
/*
 * 【说明】pthread_create(&线程id, 属性(NULL=默认), 入口函数, 参数)：
 *   成功返回 0，失败返回错误码（不是 -1，也不设置 errno，所以这里直接打印 ret）。
 *   创建成功后线程立即开始运行 isp_control_thread。
 */
td_s32 isp_control_start(void)
{
    td_s32 ret;
    /* 必须先完成ISP初始化 */
    if (g_isp_initialized == TD_FALSE) {
        printf("Initialize ISP before starting its thread\n");
        return TD_FAILURE;
    }

    /* 防止重复创建工作线程 */
    if (g_isp_thread_created == TD_TRUE) {
        printf("ISP thread already created\n");
        return TD_FAILURE;
    }
    /* 创建线程，运行ISP */
    ret = pthread_create(&g_isp_thread, TD_NULL, isp_control_thread, &g_isp_pipe);
    if (ret != 0) {
        printf("Create ISP thread failed: %d\n", ret);
        return TD_FAILURE;
    }
    g_isp_thread_created = TD_TRUE;
    printf("ISP thread created\n");

    return TD_SUCCESS;
}

/* 退出ISP并等待工作线程结束，释放已初始化资源。 */
/*
 * 【说明】退出顺序（很重要）：
 *    ① ss_mpi_isp_exit  ：通知 ISP 退出，这会让线程里阻塞的 ss_mpi_isp_run 返回
 *    ② pthread_join     ：等待线程真正结束（阻塞直到线程函数 return）
 *   不能反过来：如果先 join，线程还在 run 里阻塞，join 会永远等下去（死锁）。
 *   strerror(错误码) 把 pthread_join 返回的错误码转成可读文字，需要 <string.h>。
 */
td_s32 isp_control_deinit(void)
{
    td_s32 ret;
    ot_vi_pipe vi_pipe = 0;
    td_s32 thread_ret;

    /* 没有初始化资源，或者已经清理完成 */
    if (g_isp_mem_ready == TD_FALSE && g_isp_thread_created == TD_FALSE) {
        return TD_SUCCESS;
    }

    /* 先退出ISP，使得ss_mpi_isp_run结束运行*/
    if (g_isp_mem_ready == TD_TRUE)
    {
        ret = ss_mpi_isp_exit(vi_pipe);
        if (ret != TD_SUCCESS) {
            printf("ISP exit failed: %#x\n", (td_u32)ret);
            return ret;
        }
        g_isp_mem_ready = TD_FALSE;
        g_isp_initialized = TD_FALSE;
    }

     /* ISP已退出，再等待工作线程完全结束 */
    if (g_isp_thread_created == TD_TRUE) {

        thread_ret = pthread_join(g_isp_thread, NULL);
        if (thread_ret != 0) {
            printf("Join ISP thread failed: %s\n",
                   strerror(thread_ret));
            return TD_FAILURE;
        }

        g_isp_thread_created = TD_FALSE;
    }

    printf("ISP deinitialized\n");
    return TD_SUCCESS;
}

/* 注销AE/AWB算法和Sensor回调 */
/* 注销AE/AWB算法库及Sensor适配回调。 */
/*
 * 【说明】注销顺序与注册相反：AWB → AE → 传感器回调。
 *   调用前必须保证 ISP 已经 exit 且线程已结束（main.c 的 deinit_isp 标签里就是先 deinit 再 unregister）。
 */
void isp_control_unregister(void)
{
    td_s32 ret;
    ot_vi_pipe vi_pipe = 0;

    /*
     * 后续真正运行ISP后，必须先退出ISP并等待线程结束，
     * 然后才能调用本函数。
     */

    /* 1. 注销自动白平衡算法 */
    ret = ss_mpi_awb_unregister(vi_pipe, &g_awb_lib);
    if (ret != TD_SUCCESS) {
        printf("Unregister AWB failed: %#x\n",
               (td_u32)ret);
    }

    /* 2. 注销自动曝光算法 */
    ret = ss_mpi_ae_unregister(vi_pipe, &g_ae_lib);
    if (ret != TD_SUCCESS) {
        printf("Unregister AE failed: %#x\n",
               (td_u32)ret);
    }

    /* 3. 注销Sensor回调 */
    ret = g_sensor->pfn_un_register_callback(
        vi_pipe, &g_ae_lib, &g_awb_lib);
    if (ret != TD_SUCCESS) {
        printf("Unregister Sensor callbacks failed: %#x\n",
               (td_u32)ret);
    }
}
