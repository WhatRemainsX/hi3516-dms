/*
 * dms_alert：DMS 报警的声光输出（板载 LED + 蜂鸣器）。
 * 流水线位置：DMS 链路最末端，dms_thread 每帧把当前报警级别交给这里。
 * 硬件（迅为 TOPEET-HI3516CV610 底板 V1.2 原理图第 10 页 KEY_ADC_BEEP_TH_LED）：
 *   LED：3V3 → 1K → 绿色 LED1 → GPIO7_7（Linux GPIO 63），低电平点亮。
 *   蜂鸣器：PWM1_OUT1 → 10K → S8050 → 无源蜂鸣器 BP1；核心板上 PWM1_OUT1 网络
 *           接芯片 GPIO1_0 管脚（复用功能 7 = PWM1_OUT1），需要方波驱动。
 * 内部有一个 20ms 节拍的小线程负责闪灯和响铃节奏，不占用 DMS 处理线程的时间。
 * 任何一步初始化失败都只打印日志，对应的灯或蜂鸣器不工作，不影响 DMS 和主流程。
 */
#ifndef DMS_ALERT_H
#define DMS_ALERT_H

#include "dms_types.h"

/* 初始化 LED、蜂鸣器并启动节拍线程。失败也返回 TD_SUCCESS 以外的值但可忽略。 */
td_s32 dms_alert_init(void);

/* 停止节拍线程，灭灯、停蜂鸣器、恢复管脚复用。可重复调用。 */
void dms_alert_deinit(void);

/* 蜂鸣器模式（按报警原因决定，见 dms_thread.c）。 */
typedef enum {
    DMS_BUZZER_OFF = 0,          /* 正常：不响 */
    DMS_BUZZER_INTERMITTENT,     /* PERCLOS / 哈欠 / 分心：间断响 */
    DMS_BUZZER_CONTINUOUS        /* 长闭眼：一直响 */
} dms_buzzer_mode;

/* 设置当前报警级别（只决定 LED 节奏）。每帧调用即可。 */
void dms_alert_set_level(dms_alarm_level level);

/* 设置蜂鸣器模式。每帧调用即可。 */
void dms_alert_set_buzzer(dms_buzzer_mode mode);

#endif /* DMS_ALERT_H */
