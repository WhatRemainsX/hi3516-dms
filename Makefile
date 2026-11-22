# 交叉编译器：目标平台为 Cortex-A7，使用 musl libc。
# 工具链是 Linux x86_64 程序，Windows 主机直接跑不了，需在 Linux 容器里执行 make
# （Docker 挂载 F:\dms_project 后，$(CURDIR) 指向容器里的 new_my_camera，路径结构一致）。
CC := $(CURDIR)/../sdk_linux/Hi3516CV610_SDK_V1.0.2.0/prebuilts/gcc/linux-x86/arm/arm-v01c02-linux-musleabi-gcc/bin/arm-v01c02-linux-musleabi-gcc

# SDK根目录及头文件、库文件目录。
# 用 $(CURDIR) 相对路径指向拷过来的完整 SDK，不写 Windows 盘符，不留旧的 /mnt/data 路径。
SDK_ROOT := $(CURDIR)/../sdk_linux/Hi3516CV610_SDK_V1.0.2.0
SDK_INCLUDE := $(SDK_ROOT)/smp/a7_linux/source/out/include
SDK_LIB := $(SDK_ROOT)/smp/a7_linux/source/out/lib

# 最终生成的可执行文件。
TARGET := output/my_camera

# 应用源文件按功能分成三个子目录：
#   video/ ：采集 -> ISP -> VPSS 缩放 -> VENC 编码 -> RTSP 推流 / MP4 录像，整条"看得见画面"的链路
#   npu/   ：NPU 模型加载和推理的通用封装（svp_acl），DMS 用它跑模型
#   dms/   ：驾驶员监控
# main.c 和 app_config.h 留在根目录：main.c 是把各链路串起来的总指挥，
# app_config.h 是全局配置，不属于任何一条单独的链路。
VIDEO_SOURCES := video/media_system.c video/video_capture.c video/isp_control.c \
                 video/video_processing.c video/video_encoder.c video/media_source.c \
                 video/nal_parser.c video/rtp_h264.c video/rtp_sender.c video/rtsp_server.c \
                 video/mp4_writer.c video/mp4_recorder.c video/video_osd.c

NPU_SOURCES := npu/npu_model.c

# DMS（驾驶员监控）：在 VPSS 通道 2 的 640x640 画面上跑 SCRFD + 关键点 + 眼睛 + 头姿，
# 输出疲劳/分心状态和报警。出错只停 DMS 线程，不影响主流程。
DMS_SOURCES := dms/dms_detect.c dms/dms_models.c dms/dms_metrics.c \
               dms/dms_sink.c dms/dms_log.c dms/dms_clip.c dms/dms_occlusion.c \
               dms/dms_thread.c dms/dms_alert.c

# 应用源文件和头文件；头文件变化也会触发重新构建。
SOURCES := main.c $(VIDEO_SOURCES) $(NPU_SOURCES) $(DMS_SOURCES)
HEADERS := app_config.h \
           video/media_system.h video/video_capture.h video/isp_control.h \
           video/video_processing.h video/video_encoder.h video/media_source.h \
           video/nal_parser.h video/rtp_h264.h video/rtp_sender.h video/rtsp_server.h \
           video/mp4_writer.h video/mp4_recorder.h video/video_osd.h \
           npu/npu_model.h \
           dms/dms_types.h dms/dms_detect.h dms/dms_models.h dms/dms_metrics.h \
           dms/dms_sink.h dms/dms_log.h dms/dms_clip.h dms/dms_occlusion.h \
           dms/dms_thread.h dms/dms_alert.h

# 编译器预处理、警告、语言标准及目标CPU选项。
#   -I. -Ivideo -Inpu -Idms：
#     源文件里的 #include "xxx.h" 默认只在"当前文件所在目录"找，找不到才去 -I 指定的目录找。
#     main.c 在根目录，但要 #include "video/video_capture.h" 这种在子目录里的头文件；
#     video/ 里的 .c 又要 #include "app_config.h"（在根目录）；
#     dms/ 里的文件要 #include "npu_model.h"（在 npu/ 里）。
#     dms/ 里的头文件（dms_thread.h、dms_sink.h 等）也要能被 main.c 直接
#     #include "dms_thread.h" 找到，所以把 dms/ 也加进搜索路径。
#     所以把根目录、video/、npu/、dms/ 四个目录都加进搜索路径，各边的 #include "xxx.h" 就都能找到了。
CPPFLAGS := -I. -Ivideo -Inpu -Idms -I$(SDK_INCLUDE) -D_GNU_SOURCE -DISP_V2
# 临时附加的宏，例如 make EXTRA_DEFS="-DAPP_DMS_PROFILE=1 -DAPP_DMS_FPS=30"；平时为空。
CPPFLAGS += $(EXTRA_DEFS)

CFLAGS := -Wall -Wextra -std=gnu11
CFLAGS += -mcpu=cortex-a7 -mfloat-abi=softfp -mfpu=neon-vfpv4

# SDK媒体组件、Sensor算法库及系统库。
LDLIBS := -Wl,--start-group

LDLIBS += $(SDK_LIB)/libss_mpi.a
LDLIBS += $(SDK_LIB)/libss_mpi_sysbind.a
LDLIBS += $(SDK_LIB)/libss_mpi_sysmem.a

LDLIBS += $(SDK_LIB)/libsns_sc4336p.a
LDLIBS += $(SDK_LIB)/libss_mpi_ae.a
LDLIBS += $(SDK_LIB)/libss_mpi_awb.a
LDLIBS += $(SDK_LIB)/libss_mpi_isp.a
LDLIBS += $(SDK_LIB)/libot_mpi_isp.a

LDLIBS += $(SDK_LIB)/libdehaze.a
LDLIBS += $(SDK_LIB)/libextend_stats.a
LDLIBS += $(SDK_LIB)/libdrc.a
LDLIBS += $(SDK_LIB)/libldci.a
LDLIBS += $(SDK_LIB)/libbnr.a
LDLIBS += $(SDK_LIB)/libcalcflicker.a
LDLIBS += $(SDK_LIB)/libir_auto.a
LDLIBS += $(SDK_LIB)/libacs.a

LDLIBS += $(SDK_LIB)/libsecurec.a

# NPU 推理需要的三个库
LDLIBS += $(SDK_LIB)/libsvp_acl.a       # svp_acl_ 开头的所有函数：初始化、加载模型、执行
LDLIBS += $(SDK_LIB)/libss_mpi_km.a     # svp_acl 内部要用的库
LDLIBS += $(SDK_LIB)/libprotobuf-c.a    # svp_acl 解析 .om 文件时要用的库


LDLIBS += -Wl,--end-group
LDLIBS += -lstdc++ -lpthread -lm -ldl




# 默认目标：编译摄像头应用。
.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SOURCES) $(HEADERS)
	@mkdir -p $(dir $(TARGET))
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SOURCES) -o $(TARGET) $(LDLIBS)

# 删除构建产物。
clean:
	rm -f $(TARGET)
