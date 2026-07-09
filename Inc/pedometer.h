#ifndef __PEDOMETER_H
#define __PEDOMETER_H

#include "main.h"
#include "mpu6050.h"
#include <stdint.h>

/*
 * 计步器 — 滑动窗口动态阈值
 *
 * 调用: Pedometer_Update(&accel, &gyro) 每 50ms 一次
 */

void Pedometer_Init(void);
void Pedometer_Update(MPU6050_Accel_t *accel, MPU6050_Gyro_t *gyro);
uint32_t Pedometer_GetSteps(void);
void Pedometer_Reset(void);

extern float g_bp_debug;   /* 滤波值, 调试用 */

#endif
