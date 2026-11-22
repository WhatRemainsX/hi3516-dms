/*
 * dms_alert.c：dms_alert.h 的实现。
 *
 * 灯光（按用户要求）：
 *   无报警 NONE   → 慢闪（亮 500ms / 灭 500ms），表示 DMS 在正常运行
 *   NOTICE/WARNING → 快闪（亮 100ms / 灭 100ms）
 *   STRONG / RMF（违规、要求接管） → 常亮
 * 蜂鸣器（按报警原因，由 dms_thread 通过 dms_alert_set_buzzer 设置）：
 *   OFF          → 不响（正常）
 *   INTERMITTENT → 间断响：响 200ms / 停 300ms（PERCLOS、哈欠、头姿分心）
 *   CONTINUOUS   → 一直响（长闭眼）
 *
 * LED 用 sysfs GPIO（/sys/class/gpio/gpioN/value）。
 * 蜂鸣器用 sysfs PWM（/sys/class/pwm/pwmchipX/pwmY），先通过 /dev/mem 把 GPIO1_0
 * 管脚复用切到 PWM1_OUT1，退出时恢复原来的复用值。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "app_config.h"
#include "dms_alert.h"

#define DMS_ALERT_TICK_MS 20

static pthread_t g_alert_thread;
static td_bool g_alert_running = TD_FALSE;
static atomic_bool g_alert_stop = ATOMIC_VAR_INIT(false);
static atomic_int g_alert_level = ATOMIC_VAR_INIT(DMS_ALARM_NONE);
static atomic_int g_buzzer_mode = ATOMIC_VAR_INIT(DMS_BUZZER_OFF);

static int g_led_fd = -1;          /* /sys/class/gpio/gpioN/value */
static int g_buz_enable_fd = -1;   /* /sys/class/pwm/pwmchipX/pwmY/enable */
static td_bool g_buz_mux_saved = TD_FALSE;
static uint32_t g_buz_mux_orig = 0;

/* 往 sysfs 文件写一个字符串；返回 0 成功。 */
static int dms_alert_write_file(const char *path, const char *text)
{
    int fd = open(path, O_WRONLY);
    ssize_t n;

    if (fd < 0) {
        return -1;
    }
    n = write(fd, text, strlen(text));
    close(fd);
    return (n == (ssize_t)strlen(text)) ? 0 : -1;
}

static td_bool dms_alert_path_exists(const char *path)
{
    struct stat st;
    return (stat(path, &st) == 0) ? TD_TRUE : TD_FALSE;
}

/* 读改写一个 32 位物理寄存器（只用于蜂鸣器管脚复用）。 */
static int dms_alert_reg_rmw(uint32_t phys, uint32_t mask, uint32_t value, uint32_t *old_value)
{
    int fd;
    long page = sysconf(_SC_PAGESIZE);
    uint32_t base = phys & ~((uint32_t)page - 1u);
    volatile uint32_t *map;
    volatile uint32_t *reg;

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        return -1;
    }
    map = (volatile uint32_t *)mmap(NULL, (size_t)page, PROT_READ | PROT_WRITE,
                                    MAP_SHARED, fd, (off_t)base);
    close(fd);
    if (map == (volatile uint32_t *)MAP_FAILED) {
        return -1;
    }
    reg = (volatile uint32_t *)((volatile uint8_t *)map + (phys - base));
    if (old_value != NULL) {
        *old_value = *reg;
    }
    *reg = (*reg & ~mask) | (value & mask);
    munmap((void *)map, (size_t)page);
    return 0;
}

static void dms_alert_led(td_bool on)
{
    char c;

    if (g_led_fd < 0) {
        return;
    }
#if APP_DMS_LED_ACTIVE_LOW
    c = on ? '0' : '1';
#else
    c = on ? '1' : '0';
#endif
    (void)pwrite(g_led_fd, &c, 1, 0);
}

static void dms_alert_buzzer(td_bool on)
{
    if (g_buz_enable_fd < 0) {
        return;
    }
    (void)pwrite(g_buz_enable_fd, on ? "1" : "0", 1, 0);
}

static void dms_alert_led_init(void)
{
#if APP_DMS_LED_ENABLE
    char path[96];
    char num[16];

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", APP_DMS_LED_GPIO);
    if (dms_alert_path_exists(path) == TD_FALSE) {
        snprintf(num, sizeof(num), "%d", APP_DMS_LED_GPIO);
        (void)dms_alert_write_file("/sys/class/gpio/export", num);
    }
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", APP_DMS_LED_GPIO);
    if (dms_alert_write_file(path, "out") != 0) {
        printf("dms_alert: LED gpio%d direction failed: %s\n", APP_DMS_LED_GPIO, strerror(errno));
        return;
    }
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", APP_DMS_LED_GPIO);
    g_led_fd = open(path, O_WRONLY);
    if (g_led_fd < 0) {
        printf("dms_alert: LED gpio%d open failed: %s\n", APP_DMS_LED_GPIO, strerror(errno));
        return;
    }
    dms_alert_led(TD_FALSE);
    printf("dms_alert: LED on gpio%d ready\n", APP_DMS_LED_GPIO);
#endif
}

static void dms_alert_buzzer_init(void)
{
#if APP_DMS_BUZZER_ENABLE
    char dir[96];
    char path[128];
    char num[32];
    unsigned long period_ns = 1000000000UL / (unsigned long)APP_DMS_BUZZER_FREQ_HZ;

    /* 1. 管脚复用：GPIO1_0 → PWM1_OUT1（寄存器低 4 位是功能选择）。 */
    if (dms_alert_reg_rmw(APP_DMS_BUZZER_MUX_REG, 0xFu, APP_DMS_BUZZER_MUX_FUNC,
                          &g_buz_mux_orig) != 0) {
        printf("dms_alert: buzzer pin mux 0x%08x failed: %s\n",
               (unsigned)APP_DMS_BUZZER_MUX_REG, strerror(errno));
        return;
    }
    g_buz_mux_saved = TD_TRUE;

    /* 2. 导出 PWM 通道并设置频率（占空比 50% 方波）。 */
    snprintf(dir, sizeof(dir), "/sys/class/pwm/pwmchip%d/pwm%d",
             APP_DMS_BUZZER_PWMCHIP, APP_DMS_BUZZER_PWM_CH);
    if (dms_alert_path_exists(dir) == TD_FALSE) {
        snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/export", APP_DMS_BUZZER_PWMCHIP);
        snprintf(num, sizeof(num), "%d", APP_DMS_BUZZER_PWM_CH);
        if (dms_alert_write_file(path, num) != 0) {
            printf("dms_alert: buzzer pwm export %s failed: %s\n", path, strerror(errno));
            return;
        }
    }
    snprintf(path, sizeof(path), "%s/enable", dir);
    (void)dms_alert_write_file(path, "0");
    /* 先设 period 再设 duty（内核要求 duty <= period）。不写 duty=0：本 SDK 的
     * pwm-bsp 驱动对 duty=0 会打印警告。 */
    snprintf(path, sizeof(path), "%s/period", dir);
    snprintf(num, sizeof(num), "%lu", period_ns);
    if (dms_alert_write_file(path, num) != 0) {
        printf("dms_alert: buzzer pwm period failed: %s\n", strerror(errno));
        return;
    }
    snprintf(path, sizeof(path), "%s/duty_cycle", dir);
    snprintf(num, sizeof(num), "%lu", period_ns / 2);
    if (dms_alert_write_file(path, num) != 0) {
        printf("dms_alert: buzzer pwm duty failed: %s\n", strerror(errno));
        return;
    }
    snprintf(path, sizeof(path), "%s/enable", dir);
    g_buz_enable_fd = open(path, O_WRONLY);
    if (g_buz_enable_fd < 0) {
        printf("dms_alert: buzzer pwm enable open failed: %s\n", strerror(errno));
        return;
    }
    dms_alert_buzzer(TD_FALSE);
    printf("dms_alert: buzzer on pwmchip%d/pwm%d %dHz ready\n",
           APP_DMS_BUZZER_PWMCHIP, APP_DMS_BUZZER_PWM_CH, APP_DMS_BUZZER_FREQ_HZ);
#endif
}

static double dms_alert_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1000000.0;
}

/* 节拍线程：按当前级别计算本时刻灯和蜂鸣器应处的状态，只在状态变化时写 sysfs。 */
static void *dms_alert_main(void *arg)
{
    int cur_level = -1;
    double level_since = 0.0;
    td_bool led_state = TD_FALSE;
    td_bool buz_state = TD_FALSE;
    td_bool first = TD_TRUE;

    (void)arg;
    while (atomic_load(&g_alert_stop) == false) {
        int level = atomic_load(&g_alert_level);
        int bmode = atomic_load(&g_buzzer_mode);
        double now = dms_alert_now_ms();
        double t;
        td_bool led_on;
        td_bool buz_on = TD_FALSE;

        if (level != cur_level) {
            cur_level = level;
            level_since = now;
        }
        t = now - level_since;

        /* LED：按报警级别。 */
        switch (level) {
        case DMS_ALARM_STRONG:
        case DMS_ALARM_RMF:
            led_on = TD_TRUE;                                        /* 违规：常亮 */
            break;
        case DMS_ALARM_WARNING:
        case DMS_ALARM_NOTICE:
            led_on = ((long)(t / 100.0) % 2 == 0) ? TD_TRUE : TD_FALSE;  /* 快闪 */
            break;
        default:
            led_on = ((long)(now / 500.0) % 2 == 0) ? TD_TRUE : TD_FALSE; /* 慢闪 */
            break;
        }
        /* 蜂鸣器：按报警原因。间断节奏用绝对时间，模式切换时不会突然重置。 */
        if (bmode == DMS_BUZZER_CONTINUOUS) {
            buz_on = TD_TRUE;
        } else if (bmode == DMS_BUZZER_INTERMITTENT) {
            double ph = now - 500.0 * (double)(long)(now / 500.0);
            buz_on = (ph < 200.0) ? TD_TRUE : TD_FALSE;              /* 响 200ms / 停 300ms */
        }

        if (first || led_on != led_state) {
            dms_alert_led(led_on);
            led_state = led_on;
        }
        if (first || buz_on != buz_state) {
            dms_alert_buzzer(buz_on);
            buz_state = buz_on;
        }
        first = TD_FALSE;
        usleep(DMS_ALERT_TICK_MS * 1000);
    }
    return NULL;
}

td_s32 dms_alert_init(void)
{
    int ret;

    if (g_alert_running == TD_TRUE) {
        return TD_SUCCESS;
    }
    dms_alert_led_init();
    dms_alert_buzzer_init();
    if (g_led_fd < 0 && g_buz_enable_fd < 0) {
        printf("dms_alert: no LED/buzzer available, alert output disabled\n");
        return TD_FAILURE;
    }
    atomic_store(&g_alert_level, DMS_ALARM_NONE);
    atomic_store(&g_buzzer_mode, DMS_BUZZER_OFF);
    atomic_store(&g_alert_stop, false);
    ret = pthread_create(&g_alert_thread, NULL, dms_alert_main, NULL);
    if (ret != 0) {
        printf("dms_alert: pthread_create failed: %d\n", ret);
        dms_alert_deinit();
        return TD_FAILURE;
    }
    (void)pthread_setname_np(g_alert_thread, "dms_alert");
    g_alert_running = TD_TRUE;
    return TD_SUCCESS;
}

void dms_alert_deinit(void)
{
    if (g_alert_running == TD_TRUE) {
        atomic_store(&g_alert_stop, true);
        (void)pthread_join(g_alert_thread, NULL);
        g_alert_running = TD_FALSE;
    }
    if (g_buz_enable_fd >= 0) {
        dms_alert_buzzer(TD_FALSE);
        close(g_buz_enable_fd);
        g_buz_enable_fd = -1;
    }
    if (g_buz_mux_saved == TD_TRUE) {
        (void)dms_alert_reg_rmw(APP_DMS_BUZZER_MUX_REG, 0xFFFFFFFFu, g_buz_mux_orig, NULL);
        g_buz_mux_saved = TD_FALSE;
    }
    if (g_led_fd >= 0) {
        dms_alert_led(TD_FALSE);
        close(g_led_fd);
        g_led_fd = -1;
    }
}

void dms_alert_set_level(dms_alarm_level level)
{
    atomic_store(&g_alert_level, (int)level);
}

void dms_alert_set_buzzer(dms_buzzer_mode mode)
{
    atomic_store(&g_buzzer_mode, (int)mode);
}
