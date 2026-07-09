#ifndef __SW_CLOCK_H
#define __SW_CLOCK_H

#include "main.h"
#include <stdint.h>

/*
 * 软件钟 — 基于 SysTick 走时, 蓝牙时间同步修正
 *
 * 精度: HSE 8MHz → SysTick 1ms → 日误差约 ±1 秒
 * 回绕: SysTick 24-bit, 无符号减法自动处理
 */

/** @brief 设定基准时间, 之后 GetTime 从此开始累积 */
void SWClock_Sync(uint8_t hour, uint8_t min, uint8_t sec);

/** @brief 读取当前时间 */
void SWClock_GetTime(uint8_t *hour, uint8_t *min, uint8_t *sec);

/** @brief 是否已同步过 */
uint8_t SWClock_IsSynced(void);

#endif
