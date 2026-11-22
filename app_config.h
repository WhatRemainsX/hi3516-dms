/*
 * 【说明】app_config.h：全项目的"参数面板"。想改分辨率、帧率、码率、端口，只改这一个文件。
 * 流水线位置：所有模块进入运行前都会读取这里的编译期常量，它本身不处理数据。
 * 输入/输出：无运行时输入输出；宏值在编译时展开到采集、编码、DMS 和叠加代码。
 * 调用关系：main.c、video/、npu/、dms/ 的实现文件都直接或间接包含本文件。
 *   #define 名字 值   叫"宏"，编译前会把代码里所有的名字原样替换成值（纯文本替换，不占内存）。
 *   注意：这里改了宏，需要重新 make，所有包含它的 .c 才会更新（Makefile 已经把头文件列入依赖）。
 */

/**
 * @file app_config.h
 * @brief 摄像头应用的统一配置参数。
 *
 * 集中定义传感器输入、VPSS输出和视频编码使用的尺寸、帧率及码率。
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 传感器及 VI 输入图像的分辨率、帧率；这些值决定采集源规格。 */
/*
 * 【说明】传感器 SC4336P 的输出规格：2560x1440（400 万像素）@ 30 帧/秒。
 *   这三个值同时用在：MIPI 接收(video_capture)、VI、ISP(isp_control)、VB 缓存池(media_system)、
 *   VENC 的 GOP 计算(video_encoder)。改分辨率前要确认传感器驱动支持该模式。
 */
/* 行注释：传感器/VI 输入宽度（像素）；调大更清晰但更吃内存和带宽，改小前要确认驱动与 ISP 支持。 */
#define APP_VIDEO_WIDTH   2560
/* 行注释：传感器/VI 输入高度（像素）；调大更清晰但更吃内存和带宽，调小更省资源。 */
#define APP_VIDEO_HEIGHT  1440
/* 行注释：输入和编码帧率；调大更流畅但更耗算力和码率，调小更省资源。 */
#define APP_VIDEO_FPS     30

/* VPSS Channel 0 的缩放输出尺寸；决定 RTSP/录像实际看到的画面大小。 */
/*
 * 【说明】VPSS 缩放后的输出尺寸，也是编码器实际编码的尺寸：1280x720（720p）。
 *   比直接编码 2560x1440 省 3/4 的运算量和码率，网络传输也更稳。
 */
/* 行注释：VPSS/编码输出宽度；调大细节更多但更耗码率，调小更省算力和存储。 */
#define APP_VPSS_OUT_WIDTH  1280
/* 行注释：VPSS/编码输出高度；调大画面更细但更占带宽，调小更省资源。 */
#define APP_VPSS_OUT_HEIGHT 720

/* VENC 编码器 H.264 目标码率，单位为 kbit/s；越大越清晰、越占存储。 */
/*
 * 【说明】目标码率 2048 kbit/s = 2 Mbit/s ≈ 256 KB/秒 ≈ 每小时 900 MB。
 *   （kbit 是千"比特"，除以 8 才是千"字节"。录像容量估算就用这个数。）
 */
/* 行注释：H.264 目标码率；调大画面更清楚但更占空间，调小更容易糊。 */
#define APP_VENC_BITRATE_KBPS 2048

/* RTSP 服务端口；554 为标准端口，板端以 root 运行。 */
/*
 * 【说明】554 是 RTSP 协议的标准端口。VLC 里输入 rtsp://IP/live.sdp 不写端口号时，默认就连 554。
 *   1024 以下的端口属于"特权端口"，普通用户不能使用，所以板子上必须以 root 运行。
 */
/* 行注释：RTSP 监听端口；调成 1024 以上可让普通用户监听，调端口后客户端地址也要跟着改。 */
#define APP_RTSP_PORT 554

/* 播放地址中端口后面的路径：rtsp://<板端IP>:554/live.sdp */
/*
 * 【说明】播放地址的"路径"部分。完整地址：rtsp://192.168.1.222:554/live.sdp
 *   注意：这个宏目前只用于文档说明/将来扩展，rtsp_server.c 没有校验路径，写什么路径都能连上。
 */
/* 行注释：目前只用于说明/将来扩展；服务端不校验路径，改动不会改变实际连接行为。 */
#define APP_RTSP_PATH "/live.sdp"

/*
 * 运行期输出目录。
 * 板端已挂载 NFS 时用 "/mnt/sd/nfs"（可在 Windows 上直接看文件）；
 * 未挂载时改成 "/tmp"，否则 open() 会失败。
 */
/*
 * 【说明】调试文件(stream.h264、yuv)的输出目录。
 *   板子上把 PC 的目录通过 NFS 挂载到 /mnt/sd/nfs，这样写入板子的文件在电脑上直接能看到。
 *   如果板子没挂载，open 会失败，改成 /tmp 即可（/tmp 是内存盘，重启丢失）。
 */
/* 行注释：调试文件输出目录；调成不可写路径会打开失败，NFS 未挂载时可改 /tmp。 */
#define APP_OUTPUT_DIR "/mnt/sd/nfs"

/*
 * 【说明】这两个宏是早期"RTP 直接发给某台电脑"的测试遗留（第 5 步）：目的地址是你电脑的 IP。
 *   现在改成了 RTSP 方式，目的地址由客户端在 SETUP 时的 client_port + 连接来源 IP 自动决定，
 *   所以 main.c 不再使用它们，可以保留也可以删除。
 */
/* 行注释：早期 RTP 测试目的 IP；当前 RTSP 流程未使用，改动不影响现有连接。 */
#define APP_RTP_DST_IP "192.168.1.118"

/* 行注释：早期 RTP 测试端口；当前 RTSP 流程未使用，改动不影响现有连接。 */
#define APP_RTP_PORT   5004

/* 早期录像测试参数。当前 main.c 传入 0，常规录像持续到程序退出。 */
#define APP_REC_SECONDS 10

/* ============================================================
 * DMS（驾驶员监控）参数
 * ------------------------------------------------------------
 * 疲劳检测：PERCLOS / 长闭眼 / 打哈欠；分心检测：头姿。
 * DMS 通道尺寸和帧率由 video_processing.c 配置。模型、判定阈值、
 * 四级视线脱离报警和事件录像参数由 dms/ 下的模块使用。
 * ============================================================ */

/* 性能统计开关：1 = DMS 线程每 5 秒打印一行各步骤耗时和 CPU 占用（测性能用）。
 * 正式版保持 0，不产生任何额外代码。也可以用 make EXTRA_DEFS=-DAPP_DMS_PROFILE=1 临时打开。 */
#ifndef APP_DMS_PROFILE
#define APP_DMS_PROFILE 0
#endif

/* 0 使 dms_thread_start() 提前返回，跳过模型加载和线程创建。 */
#define APP_DMS_ENABLE 1

/* DMS 专用的 VPSS 输出通道号（通道 0 给编码，通道 1 不用，通道 2 给 DMS）。 */
/* 行注释：DMS 用的 VPSS 通道；调错通道会取不到 640x640 画面。 */
#define APP_DMS_VPSS_CHN 2

/* DMS 输入画面尺寸。SCRFD-500M 的 ONNX 输入是固定的 1x3x640x640，
 * 所以这里用 640x640，由 VPSS 硬件缩放+裁剪完成，CPU 不做缩放。
 * 注意（2026-11-20 板端实测）：Hi3516CV610 的 VPSS 通道裁剪是在“缩放之后”、
 * 按通道输出坐标做的，不是在 2560x1440 原图上做。所以通道 2 先等比缩放到
 * APP_DMS_SCALED_WIDTH x 640（1136x640，像素为正方形），再从中间裁 640x640，
 * 等价于原图中央约 1440x1440 的正方形，不再纵向拉伸。
 * 关键点和框都在同一个 640x640 坐标系里，互相一致，不影响 EAR/MAR 的比值。 */
/* 通道 2 中心裁剪开关：1 = 先等比缩放，再取中间的 640x640；
 * 0 = 整幅 2560x1440 直接拉伸到 640x640（旧行为，纵向拉伸 1.78 倍，视野最宽）。
 * PnP 内参 fx/fy 会随这个开关自动切换（dms_models.c）。
 * 用 #ifndef 包起来是为了构建时可用 -DAPP_DMS_CENTER_CROP=0 出对照版本。 */
#ifndef APP_DMS_CENTER_CROP
#define APP_DMS_CENTER_CROP 1
#endif
/* 中心裁剪的水平偏移（原图像素，相对画面正中；正数往右、负数往左），
 * 范围 -560~+560。驾驶员坐在画面一侧时调它，让裁剪框套住驾驶员的脸。
 * 只在 APP_DMS_CENTER_CROP=1 时生效；超范围编译时报错。 */
#ifndef APP_DMS_CROP_OFFSET_X
#define APP_DMS_CROP_OFFSET_X 0
#endif
/* 通道 2 等比缩放后的宽度：2560*640/1440=1137.8，按 16 对齐取 1136。 */
#define APP_DMS_SCALED_WIDTH \
    ((APP_VIDEO_WIDTH * APP_DMS_IN_HEIGHT / APP_VIDEO_HEIGHT) & ~15)
/* 在缩放后画面里的裁剪起点 x（4 对齐）；偏移为 0 时 = (1136-640)/2 = 248。 */
#define APP_DMS_CROP_X \
    (((APP_DMS_SCALED_WIDTH - APP_DMS_IN_WIDTH) / 2 + \
      APP_DMS_CROP_OFFSET_X * APP_DMS_IN_HEIGHT / APP_VIDEO_HEIGHT) & ~3)

/* 行注释：DMS 模型输入宽度；必须与 SCRFD 固定输入一致，调错会加载失败。 */
#define APP_DMS_IN_WIDTH   640
/* 行注释：DMS 模型输入高度；关键点/框/坐标公式都按这个尺寸算，调错会失配。 */
#define APP_DMS_IN_HEIGHT  640

#if APP_DMS_CENTER_CROP && \
    (APP_DMS_CROP_X < 0 || APP_DMS_CROP_X > APP_DMS_SCALED_WIDTH - APP_DMS_IN_WIDTH)
#error "APP_DMS_CROP_OFFSET_X out of range"
#endif

/* DMS 线程的取帧帧率。SCRFD 每 APP_DMS_DETECT_EVERY 帧跑一次，中间帧只跟踪。 */
/* 行注释：DMS 取帧频率；调高更流畅但更耗时，调低响应变慢且窗口样本变少。 */
#ifndef APP_DMS_FPS
#define APP_DMS_FPS 12
#endif
/* 行注释：每几帧跑一次检测；调大更省 NPU，调小检测更频繁但跟踪更抖。 */
#define APP_DMS_DETECT_EVERY 3

/* 3 个模型的 .om 路径（由 ONNX 转好之后放到板子上）。
 * 关键点模型已从 facial-landmarks-35-adas-0002（35 点，60x60）换成
 * PFLD 98 点（pfld_98.onnx → pfld_98.om，112x112，BGR，像素/255，
 * 检测框长边 ×1.1 的正方形裁剪，见 dms_models.c）。 */
/* 行注释：人脸检测模型路径；改为不存在或不匹配的模型会使 DMS 初始化失败。 */
#define APP_DMS_DET_MODEL_PATH   "/mnt/sd/nfs/dms/scrfd_500m.om"
/* 行注释：98 点关键点模型路径；改为其他输入尺寸会不匹配，导致 DMS 初始化失败。 */
#define APP_DMS_LM_MODEL_PATH    "/mnt/sd/nfs/dms/pfld_98.om"
/* 行注释：眼睛开闭分类模型路径；改为不存在或不匹配的模型会使眼睛状态不可用。
 * 必须是用 (v-127)/255 量程校准重转的版本（2026-11-20 起），旧的 open_closed_eye_0001.om
 * 是按 0~1 量程转的，和现在的预处理不匹配。 */
#define APP_DMS_EYE_MODEL_PATH   "/mnt/sd/nfs/dms/open_closed_eye_m127.om"
/* 已不用：head-pose-estimation-adas-0001 的 Caffe 权重官方从未公开，无法转 .om。
 * 头姿改用 98 点关键点的 PnP 解算（dms_models_run_pose），此宏仅保留占位。 */
/* 行注释：旧头姿模型占位路径；当前 PnP 实现未使用，改动不影响当前头姿结果。 */
#define APP_DMS_POSE_MODEL_PATH  "/mnt/sd/nfs/dms/head_pose_estimation_adas_0001.om"

/* 检测阈值：分数决定什么算人脸，NMS 决定重叠框保留多少。 */
/* 行注释：SCRFD 人脸分数门槛；调高减少误检但可能漏掉侧脸，调低误检增多。 */
#define APP_DMS_SCORE_THRESH 0.5f
/* 行注释：NMS 重叠门槛；调高允许更多重叠框，调低更快合并近邻框。 */
#define APP_DMS_NMS_THRESH   0.4f
/* 人脸框外扩比例：外扩后的框只写进 dms_face_state.box 做记录。
 * 注意：关键点裁剪已不用它（2026-11-20 起直接用检测原始框 ×1.1，见 dms_models.c），
 * 改这个值不影响关键点和判定。 */
/* 行注释：记录用的人脸框外扩比例；不影响关键点和判定。 */
#define APP_DMS_BOX_EXPAND   0.15f
/* 跟踪匹配的 IoU 门限，以及框平滑系数（越大越平滑、延迟越大）。 */
/* 行注释：跟踪匹配 IoU 门槛；调高更难跟上快速转头，调低更容易接错人。 */
#define APP_DMS_TRACK_IOU    0.3f
/* 行注释：旧框平滑权重；调大更稳定但延迟更大，调小更跟手但会抖。 */
#define APP_DMS_BOX_SMOOTH   0.6f

/* 眼睛：闭眼判定 = 分类器两只眼都判闭  且  EAR < 本人睁眼基线 × APP_DMS_EAR_CLOSED_RATIO。
 * 2026-11-20 用板端事件录像回放标定：戴眼镜、低机位下分类器单独用会把约 16% 的睁眼帧
 * 判闭（镜片反光），PFLD 的 EAR 单独用睁闭差距又小（睁眼约 0.28~0.33、闭眼约 0.21~0.23）；
 * 两者“且”起来后 258 帧睁眼零误报，已知闭眼帧全部检出。
 * 睁眼基线开机/换人后从睁眼帧自动学习，之后缓慢跟随。 */
/* 行注释：睁眼 EAR 基线的初值（学到真实值之前用）；一般不用改。 */
#define APP_DMS_EAR_OPEN     0.29f
/* 行注释：EAR 低于基线的多少倍才认可分类器的“闭眼”；调大更容易判闭，调小更保守。 */
#define APP_DMS_EAR_CLOSED_RATIO 0.88f
/* 行注释：EAR 绝对兜底线（保留宏，当前判定不用）。 */
#define APP_DMS_EAR_CLOSED   0.15f

/* 打哈欠：嘴纵横比 MAR > 门槛 且持续 1 秒算一次；60 秒内 3 次报警。
 * MAR = 内唇上下中点距离 / 两嘴角距离（98 点 90-94 / 76-82）。门槛按板端实测标定
 * （2026-11-21 日志）：闭嘴/说话 0.01~0.20，真实打哈欠持续 0.27~0.37，
 * 按这个定义 MAR 到不了 0.6，所以门槛取 0.25。换关键点模型或改 MAR 公式后要重标。 */
/* 行注释：打哈欠的 MAR 门槛；调低更容易算哈欠，调高则普通张嘴不报。 */
#define APP_DMS_MAR_YAWN     0.25f
/* 行注释：转头超过这个角度（相对基线 |yaw|）时不计哈欠：侧脸时嘴宽被压缩，MAR 会虚高。 */
#define APP_DMS_YAWN_MAX_YAW 20.0f
/* 行注释：MAR 超门槛需保持的毫秒数；调短更快计一次哈欠，调长更抗瞬时误判。 */
#define APP_DMS_YAWN_HOLD_MS 1000
/* 行注释：窗口内多少哈欠触发疲劳；调小更快报警但更易误报，调大更保守。 */
#define APP_DMS_YAWN_COUNT   3
/* 行注释：哈欠计数窗口毫秒数；调大统计更多旧事件也更容易累积，调小只看最近短时间。 */
#define APP_DMS_YAWN_WIN_MS  60000

/* PERCLOS：60 秒窗口内闭眼样本占比 > 0.15 判疲劳（降到 0.12 以下才解除；开机/换人后
 * 攒满 30 秒数据才开始判）；单次闭眼超过 APP_DMS_LONG_BLINK_MS 也判疲劳。 */
/* 行注释：PERCLOS 统计窗口；调大更平滑但报警变慢，调小对短时闭眼更敏感。 */
#define APP_DMS_PERCLOS_WIN_MS  60000
/* 行注释：闭眼占比报警线；调高更保守，调低更容易判疲劳。 */
#define APP_DMS_PERCLOS_THRESH  0.15f
/* 行注释：长闭眼持续时长；调长更不容易误报，调短更早发长闭眼事件。
 * 2026-11-20 板端实测：400ms 时低机位+往下看屏幕造成的“眯眼”约 70 秒报了 17 次；
 * 这些误判段一般只有 0.4~1 秒，真正打盹闭眼远长于此，曾改为 1000ms；之后按用户要求
 * 恢复 400ms（长闭眼时蜂鸣器一直响）。若误报太多，可调回 1000。
 */
#define APP_DMS_LONG_BLINK_MS    400

/* 分心：相对“本人正常姿态基线”的 |yaw| > 30 度（左右转头），或 pitch < -20 度（低头），
 * 持续 3.5 秒。摄像头一般装在侧前方/下方，驾驶员正常看路时相对摄像头本来就有
 * 几十度的 yaw/pitch，所以必须减去基线再比阈值（2026-11-20 板端实测正常坐姿
 * yaw≈-30、pitch≈+16，不减基线就会一直报分心）。 */
/* 行注释：开机/换人后用多少帧有效头姿求基线（12fps 下 36 帧≈3 秒），期间不判分心。 */
#define APP_DMS_POSE_CALIB_FRAMES 36
/* 行注释：基线缓慢跟随的时间常数（秒），只在未越界时跟随；调大更稳，调小适应坐姿变化更快。 */
#define APP_DMS_POSE_ADAPT_SECONDS 30
/* 行注释：连续无人脸超过这么久就认为可能换人，重新标定基线。 */
#define APP_DMS_POSE_RECALIB_LOST_MS 10000
/* 行注释：水平偏头角度门槛；调大更不容易判分心，调小则轻微偏头也可能报警。 */
#define APP_DMS_YAW_THRESH       30.0f
/* 行注释：低头角度门槛（越负越不像低头）；调大更接近 0 更敏感，调小更负则更不容易报警。 */
#define APP_DMS_PITCH_THRESH     (-20.0f)
/* 行注释：头姿越界持续多久才判分心；调长更抗短暂转头，调短反应更快。 */
#define APP_DMS_DISTRACT_HOLD_MS 3500
/* 行注释：分心退出的回差（度）：进入按上面门槛，退出要回到 |yaw|<25°、pitch>-15°，
 * 避免角度在门槛附近来回跳时分心状态反复开关。调大更难退出。 */
#define APP_DMS_DISTRACT_EXIT_MARGIN 5.0f

/* ============================================================
 * 分级报警时序（GB 47955-2026 视线脱离 EOR 链，四级）
 *   “视线脱离” = 长闭眼 或 头姿分心（相对基线越界持续 3.5 秒） 或 曾有人脸后连续无人脸 3.5 秒。
 *   从视线脱离被确认（≤5 秒，满足标准“5 秒内发 EOR”）开始：
 *     L1 NOTICE  ：EOR 视线脱离提示
 *     L2 WARNING ：+3 秒  升级 EOR
 *     L3 STRONG  ：再 +5 秒 DCA 驾驶员接管需求
 *     L4 RMF     ：再 +10 秒 RMF 最小风险策略（实车由底盘执行减速停车；本板只输出状态）
 *   视线脱离条件连续消失 APP_DMS_EOR_RELEASE_MS 后复位。
 *   疲劳（PERCLOS 超标、哈欠过多）不属于 EOR 链：条件成立期间 L1，持续 3 秒后 L2。
 * ============================================================ */
/* 行注释：L1 → L2（升级 EOR）间隔，标准 3 秒。 */
#define APP_DMS_EOR_UPGRADE_MS  3000
/* 行注释：L2 → L3（DCA）间隔，标准 5 秒。 */
#define APP_DMS_EOR_DCA_MS      5000
/* 行注释：L3 → L4（RMF）间隔，标准 ≤10 秒。 */
#define APP_DMS_EOR_RMF_MS      10000
/* 行注释：视线回到路面要连续保持这么久，EOR 链才复位（L 等级回 L0）。
 * 防止闭眼/分心检测偶尔断一两帧就把 L2/L3 打回 L0；调大更不容易复位，调小复位更快。 */
#define APP_DMS_EOR_RELEASE_MS  2000
/* 行注释：疲劳类报警 L1 → L2 的持续时间。 */
#define APP_DMS_ALARM_L2_MS     3000
/* 行注释：车速门限（km/h）。接入车速后，车速 ≤ 此值时不运行 EOR 链；未接车速（-1）时始终运行。 */
#define APP_DMS_MIN_SPEED_KMH   10.0f

/* 状态日志打印周期（毫秒）；只影响日志/叠加频率，不改变判定。 */
/* 行注释：状态日志/叠加刷新周期；调长打印更少但显示更新慢，调短更实时但更占串口。 */
#define APP_DMS_LOG_INTERVAL_MS 1000

/* dms_clip.c 从缓存内可用的 H.264 关键帧开始写，再继续写触发后的画面。 */
/* 行注释：事件录像总开关；调成 0 停止事件录像，调成 1 启用触发前/后录像。 */
#define APP_DMS_CLIP_ENABLED      1
/* 预录缓冲和关键帧查找窗口的基准时长。实际预录时长由关键帧位置决定。 */
#define APP_DMS_CLIP_PRE_SECONDS  10
/* 行注释：触发后续录秒数；调大录像更长且更占卡，调小更快收尾。 */
#define APP_DMS_CLIP_POST_SECONDS 10
/* 行注释：事件录像目录/文件名模板；调成不可写路径会开文件失败，调成其他目录则换存储位置。 */
#define APP_DMS_CLIP_PATH         "/mnt/sd/nfs/dms/dms_event.mp4"
/* 关闭当前片段后的冷却时间，约束新文件的创建。录像中的事件仍可延长当前片段。 */
#define APP_DMS_CLIP_COOLDOWN_SECONDS 60
/*
 * 触发前录像环形缓冲的字节上限。按 2Mbit/s 码率估算，10 秒约 2.5MB，
 * 留出余量取 4MB；超限时丢最老的帧，硬上限由预分配字节池保证。
 */
/* 行注释：预录环形池硬上限；调大保留更多帧但吃 RAM，调小可能挤掉更老的预录帧。 */
#define APP_DMS_CLIP_RING_MAX_BYTES (4 * 1024 * 1024)

/* RTSP 叠加画字开关：0 = 只缓存文字、不画画；
 * 1 = 用 SDK REGION 在 VENC 通道 0 上画字（OVERLAY 句柄 0，1Hz 更新位图）。
 * 实现见 video/video_osd.c：内置 8x8 点阵字体，ARGB1555 位图 1024x8，
 * 挂在 VENC 通道 0 的 (16,16) 位置；RGN 失败只记日志，不影响主流程。 */
/* 行注释：RGN 文字叠加开关；调成 0 不画字，调成 1 才在编码画面上显示文字。 */
#define APP_DMS_OSD_RGN 1

/* ============================================================
 * 报警声光输出（dms/dms_alert.c）。硬件依据：迅为 TOPEET-HI3516CV610 底板 V1.2
 * 原理图第 10 页（KEY_ADC_BEEP_TH_LED）+ 核心板 V1.1 原理图第 5 页。
 *   灯：无报警慢闪 / NOTICE、WARNING 快闪 / STRONG（违规）常亮。
 *   蜂鸣器按报警原因（不按级别）：
 *     正常不响；PERCLOS 超标 / 哈欠超标 / 头姿分心 → 间断响；长闭眼 → 一直响。
 * ============================================================ */
/* 行注释：LED 总开关；0 = 不控制 LED。 */
#define APP_DMS_LED_ENABLE        1
/* 行注释：LED 所在 GPIO（GPIO7_7 = 7*8+7 = 63）。 */
#define APP_DMS_LED_GPIO          63
/* 行注释：LED 低电平点亮（3V3→1K→LED→GPIO），板上实测确认。 */
#define APP_DMS_LED_ACTIVE_LOW    1
/* 行注释：蜂鸣器总开关；0 = 不响。板载是无源蜂鸣器（板上实测：拉高只“咔”一声），必须 PWM 驱动。 */
#define APP_DMS_BUZZER_ENABLE     1
/* 行注释：PWM1 控制器在 sysfs 里是 pwmchip3（pwmchip0 是 SVB 调压用的 PWM0，不能动）。 */
#define APP_DMS_BUZZER_PWMCHIP    3
/* 行注释：PWM1_OUT1 = 通道 1。 */
#define APP_DMS_BUZZER_PWM_CH     1
/* 行注释：蜂鸣器频率（Hz）；按用户要求 1000Hz。 */
#define APP_DMS_BUZZER_FREQ_HZ    1000
/* 行注释：GPIO1_0 管脚复用寄存器 io1_cfg_reg15，低 4 位 = 7 时为 PWM1_OUT1（芯片引脚表）。 */
#define APP_DMS_BUZZER_MUX_REG    0x1113003Cu
#define APP_DMS_BUZZER_MUX_FUNC   7u

/* 摄像头遮挡自检：亮度方差阈值（TODO：实车标定，当前只是占位）。 */
/* 行注释：摄像头遮挡亮度方差门槛（当前占位）；调大更不容易报遮挡，调小更容易报。 */
#define APP_DMS_OCCLUSION_VAR_THRESH 40.0f

#endif
