#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

/*
 * MPU6500/MPU6050 I2C 驱动接口
 *
 * 硬件: I2C2, PB10(SCL) + PB11(SDA), 400kHz
 * 器件: 实际硅片为 MPU6500 (WHO_AM_I=0x70)，寄存器与 MPU6050 完全兼容
 * 地址: 0x68 (AD0 引脚接地)
 *
 * 使用流程:
 *   1. MX_I2C2_Init()          -- CubeMX 生成的 I2C2 初始化
 *   2. MPU6050_Init()          -- 检查 WHO_AM_I → 唤醒 → 配置量程/采样率
 *   3. MPU6050_ReadAccel()     -- 循环读取，200ms 间隔
 *   4. MPU6050_CalcAngle()     -- 从加速度算俯仰/横滚角
 *   5. g_mpu6050_i2c_error     -- 检查通信是否异常
 */

/* ==================== I2C 地址 ==================== */

/* 7-bit I2C 地址: 0x68 (AD0=GND) 或 0x69 (AD0=VCC)
 * HAL 库调用时需左移 1 位: MPU6050_ADDR << 1
 * 本模块的 AD0 已接地，地址确认为 0x68 */
#define MPU6050_ADDR            0x68

/* ==================== 寄存器地址表 ==================== */

/* ---- 器件识别 ---- */
#define MPU6050_REG_WHO_AM_I    0x75  /* RO, 复位值: MPU6050=0x68, MPU6500=0x70 */

/* ---- 电源管理 ---- */
#define MPU6050_REG_PWR_MGMT_1  0x6B  /* RW, bit6=SLEEP, bit[2:0]=CLKSEL */

/* ---- 采样率分频 ---- */
#define MPU6050_REG_SMPLRT_DIV  0x19  /* RW, 采样率 = 1kHz / (1 + DIV) */

/* ---- 低通滤波器 ---- */
#define MPU6050_REG_CONFIG      0x1A  /* RW, bit[2:0]=DLPF_CFG, 控制加速度和陀螺的带宽 */

/* ---- 陀螺仪量程 ---- */
#define MPU6050_REG_GYRO_CONFIG 0x1B  /* RW, bit[4:3]=FS_SEL: 00=±250, 01=±500, 10=±1000, 11=±2000 °/s */

/* ---- 加速度计量程 ---- */
#define MPU6050_REG_ACCEL_CONFIG 0x1C /* RW, bit[4:3]=AFS_SEL: 00=±2g, 01=±4g, 10=±8g, 11=±16g */

/* ---- 传感器数据寄存器 (均为 RO, 16bit 大端, 高字节在前) ---- */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B /* Accel X 高 8 位, 连续读 6 字节获取 X/Y/Z */
#define MPU6050_REG_ACCEL_YOUT_H 0x3D
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F
#define MPU6050_REG_TEMP_OUT_H  0x41 /* 温度高 8 位, 连续读 2 字节 */
#define MPU6050_REG_GYRO_XOUT_H 0x43 /* Gyro X 高 8 位, 连续读 6 字节获取 X/Y/Z */
#define MPU6050_REG_GYRO_YOUT_H 0x45
#define MPU6050_REG_GYRO_ZOUT_H 0x47

/* ==================== 数据结构 ==================== */

/** @brief 加速度数据, 单位 m/s²
 *  量程 ±2g → 范围约 ±19.62 m/s²
 *  Z 轴静止时约 +9.81 (重力方向) */
typedef struct {
    float ax;       /* X 轴加速度, m/s^2 */
    float ay;
    float az;
} MPU6050_Accel_t;

/** @brief 陀螺仪数据, 单位 °/s
 *  量程 ±250 °/s
 *  静止时三轴均接近 0 */
typedef struct {
    float gx;       /* 绕 X 轴角速度, °/s */
    float gy;
    float gz;
} MPU6050_Gyro_t;

/** @brief 姿态角, 单位度
 *  由加速度数据通过 atan2 计算, 仅适用于静止或匀速状态
 *  (加速运动时加速度计测量的是"比力"而非纯重力方向) */
typedef struct {
    float pitch;    /* 俯仰角: 绕 X 轴旋转, 前倾为正, 范围 -90° ~ +90° */
    float roll;     /* 横滚角: 绕 Y 轴旋转, 右倾为正, 范围 -180° ~ +180° */
} MPU6050_Angle_t;

/* ==================== 驱动 API ==================== */

/** @brief 初始化 MPU6500/6050
 *  ① 读 WHO_AM_I 验证器件存在 (接受 0x68 或 0x70)
 *  ② 唤醒芯片 (清除 SLEEP 位)
 *  ③ 配置采样率 200Hz
 *  ④ 配置 DLPF 低通滤波 42Hz
 *  ⑤ 设置陀螺仪量程 ±250 °/s
 *  ⑥ 设置加速度计量程 ±2g
 *  @return 0=成功, -1=失败 (WHO_AM_I 不匹配或 I2C 通信错误) */
int MPU6050_Init(void);

/** @brief 检查 MPU6500/6050 是否在线
 *  读 WHO_AM_I 寄存器验证 (仅接受 0x68，用于兼容旧检查逻辑)
 *  @return 0=在线, -1=离线 */
int MPU6050_CheckConnection(void);

/** @brief 读取三轴加速度
 *  从 ACCEL_XOUT_H 起连续读 6 字节 → 拼合成 3 个 int16 → 转换为 m/s²
 *  @param accel 输出: 加速度数据结构指针
 *  @return 0=成功, -1=I2C 通信失败 */
int MPU6050_ReadAccel(MPU6050_Accel_t *accel);

/** @brief 读取三轴陀螺仪
 *  从 GYRO_XOUT_H 起连续读 6 字节 → 拼合成 3 个 int16 → 转换为 °/s
 *  @param gyro 输出: 陀螺仪数据结构指针
 *  @return 0=成功, -1=I2C 通信失败 */
int MPU6050_ReadGyro(MPU6050_Gyro_t *gyro);

/** @brief 读取芯片温度
 *  从 TEMP_OUT_H 起连续读 2 字节 → 拼合为 int16
 *  公式: T(°C) = raw / 340.0 + 36.53
 *  @return 温度 (°C), I2C 失败时返回 -273.0f (哨兵值, 低于绝对零度) */
float MPU6050_ReadTemp(void);

/** @brief 从加速度数据计算俯仰角和横滚角
 *  基于重力矢量在三个轴上的分量, 使用 atan2 计算角度
 *  前提: 传感器处于静止或匀速运动状态 (加速度=重力)
 *       加速/振动时计算结果包含运动加速度, 角度不可靠
 *  @param accel 输入: 加速度数据
 *  @param angle 输出: 俯仰角和横滚角 (°) */
void MPU6050_CalcAngle(MPU6050_Accel_t *accel, MPU6050_Angle_t *angle);

/** @brief 读 WHO_AM_I 寄存器原始值
 *  调试用——正常使用应调用 MPU6050_Init, 无需单独读此寄存器
 *  @return WHO_AM_I 值 (0x68=6050, 0x70=6500), I2C 失败时返回 0x00 */
uint8_t MPU6050_WhoAmI(void);

/* ==================== 全局调试标志 ==================== */

/* 任何 I2C 读写失败时置 1, 由调用方读取后决定是否重试/告警 */
extern uint8_t g_mpu6050_i2c_error;

/* HAL 状态码: 0=HAL_OK, 1=HAL_ERROR, 2=HAL_BUSY, 3=HAL_TIMEOUT
 * 仅在 I2C 出错时更新, 成功时保持上次值 */
extern uint8_t g_mpu6050_debug;

/* 最近一次 WHO_AM_I 读取的原始值, 调试时用于确认芯片型号 */
extern uint8_t g_mpu6050_whoami;

#endif /* __MPU6050_H */
