/**
 * 计步器 — 滑动窗口动态阈值
 *
 * L2范数 → IIR滤波 → 滑动窗口 max/min → 上升沿波峰计步
 * 采样率: 20Hz (50ms)
 */

#include "pedometer.h"
#include "cmsis_os.h"
#include <math.h>

#define FILT_ALPHA      0.78f   /* filtered = 0.78*old + 0.22*new, fc≈1.2Hz@20Hz, 更强平滑 */
#define THRESH_WIN      50      /* 滑动窗口: 50 样本 (2.5s) */
#define STEP_MIN_MS     250     /* 两步最小间隔, 防重复触发 */
#define DEAD_ZONE       0.35f   /* 死区, 信号需显著越过中线才算 */
#define MIN_SWING       0.85f   /* 窗口摆幅低于此不计步 (主防抖, 越大越不灵敏) */

static float    g_filtered;
static float    g_win[THRESH_WIN];
static uint8_t  g_win_idx;
static uint8_t  g_win_full;
static uint32_t g_steps;
static uint32_t g_last_step_ms;
static uint8_t  g_above;

float g_bp_debug;

void Pedometer_Init(void) {
    g_filtered     = 9.81f;
    g_win_idx      = 0;
    g_win_full     = 0;
    g_steps        = 0;
    g_last_step_ms = 0;
    g_above        = 0;
}

void Pedometer_Reset(void)   { g_steps = 0; }
uint32_t Pedometer_GetSteps(void) { return g_steps; }

void Pedometer_Update(MPU6050_Accel_t *accel, MPU6050_Gyro_t *gyro) {
    (void)gyro;
    uint32_t now = osKernelSysTick();

    /* 合成幅值 */
    float mag = sqrtf(accel->ax * accel->ax +
                      accel->ay * accel->ay +
                      accel->az * accel->az);

    /* IIR 滤波 */
    g_filtered = FILT_ALPHA * g_filtered + (1.0f - FILT_ALPHA) * mag;
    g_bp_debug = g_filtered;

    /* 环形缓冲 */
    g_win[g_win_idx] = g_filtered;
    g_win_idx = (g_win_idx + 1) % THRESH_WIN;
    if (g_win_full < THRESH_WIN) g_win_full++;

    /* 窗口未满时不计步 */
    if (g_win_full < 30) return;

    /* 遍历窗口找 max/min */
    float max_v = g_win[0], min_v = g_win[0];
    for (uint8_t i = 1; i < g_win_full; i++) {
        if (g_win[i] > max_v) max_v = g_win[i];
        if (g_win[i] < min_v) min_v = g_win[i];
    }

    /* 摆幅不足 → 静止, 不计步 */
    if ((max_v - min_v) < MIN_SWING) {
        g_above = 0;
        return;
    }

    float threshold = (max_v + min_v) / 2.0f + DEAD_ZONE;

    /* 上升沿穿越阈值 → 计步 */
    if (g_filtered > threshold && !g_above) {
        if ((now - g_last_step_ms) > STEP_MIN_MS) {
            g_steps++;
            g_last_step_ms = now;
        }
        g_above = 1;
    }
    if (g_filtered < threshold) {
        g_above = 0;
    }
}
