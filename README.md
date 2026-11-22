# Hi3516CV610 DMS — 驾驶员监测系统

基于海思 Hi3516CV610（20S，1 TOPS NPU）的边缘端驾驶员监测系统（DMS），在人脸识别相机工程基础上扩展。

## 功能

- **疲劳检测**：闭眼判定（眼分类器 + EAR 个人基线联合）、长闭眼（400ms）、PERCLOS（60s 窗口，0.15 阈值带回差）、哈欠检测（MAR，板端实测标定阈值 0.25）
- **分心检测**：纯 C 实现的 PnP 头姿估计（98 点关键点 → 高斯牛顿迭代求解 yaw/pitch/roll），相对个人姿态基线判定，带退出回差
- **报警链**：按 GB 47955 的四级时序（NOTICE → WARNING → DCA → RMF），LED（GPIO）+ 蜂鸣器（PWM）声光报警，EOR 复位带 2000ms 延时防抖
- **视频链路**：VI → ISP → VPSS（Ch0 录像/RTSP、Ch2 DMS 640×640@12fps）→ VENC H.264，RTSP 推流、MP4 录像、事件触发预录（前 10s + 后 10s）
- **模型**：SCRFD-500M 人脸检测 + PFLD 98 点关键点 + 眼睛开闭分类，ATC 转换为 .om 在 NPU 上推理

## 目录结构

- `main.c` — 程序入口
- `app_config.h` — 全部参数与开关集中配置
- `video/` — 视频主链路（采集、VPSS、编码、OSD、RTSP、MP4）
- `dms/` — DMS 主体（线程、模型推理、指标判定、报警、事件录像）
- `npu/` — NPU 模型加载与推理封装
- `函数速查.md` — 代码阅读顺序与函数速查

## 构建

需要海思 Hi3516CV610 SDK 与 ARM 交叉编译工具链：

```bash
make
# 打开性能统计（每 5s 打印各步耗时）
make EXTRA_DEFS=-DAPP_DMS_PROFILE=1
```

模型文件（.om）需用 ATC 转换后放到板端 `/mnt/sd/nfs/dms/` 下，不在仓库内。
