/**
 * MPU6500/MPU6050 驱动实现
 *
 * 通信: I2C2, 400kHz, PB10(SCL)+PB11(SDA)
 * 芯片: 实际为 MPU6500 (WHO_AM_I=0x70)，寄存器与 MPU6050 完全兼容
 *
 * 数据读取流程:
 *   I2C START → 器件地址+W → 寄存器地址 → RESTART → 器件地址+R → 读数据 → STOP
 *
 * 寄存器均为 16bit 大端 (MSB 在前), 需手动拼合: val = (high << 8) | low
 */

#include "mpu6050.h"
#include "i2c.h"
#include <math.h>

/*
 * ==================== 量程与换算因子 ====================
 *
 * 这些常量的取值取决于 Init 中配置的量程范围。
 * 修改量程时必须同步修改这些因子。
 */

/*
 * 加速度计换算因子: 将原始 int16 转换为 m/s²
 *
 * 当前配置: ±2g 量程 (AFS_SEL=0)
 *   灵敏度 = 16384 LSB/g  (数据手册: ACCEL_CONFIG register)
 *   1 LSB = 1/16384 g
 *   1 g = 9.81 m/s²
 *   → 1 LSB = 9.81 / 16384 m/s²
 *
 * 其他量程的灵敏度:
 *   ±4g  → 8192 LSB/g
 *   ±8g  → 4096 LSB/g
 *   ±16g → 2048 LSB/g
 */
#define ACCEL_SCALE_FACTOR      (9.81f / 16384.0f)

/*
 * 陀螺仪换算因子: 将原始 int16 转换为 °/s
 *
 * 当前配置: ±250 °/s 量程 (FS_SEL=0)
 *   灵敏度 = 131 LSB/(°/s)  (数据手册: GYRO_CONFIG register)
 *   1 LSB = 1/131 °/s
 *
 * 其他量程的灵敏度:
 *   ±500  → 65.5 LSB/(°/s)
 *   ±1000 → 32.8 LSB/(°/s)
 *   ±2000 → 16.4 LSB/(°/s)
 */
#define GYRO_SCALE_FACTOR       (1.0f / 131.0f)

/*
 * 温度换算: 将原始 int16 转换为 °C
 *
 * 数据手册公式: Temperature(°C) = (TEMP_OUT / 340.0) + 36.53
 *   TEMP_OUT 是有符号数 (int16)
 *   340.0 = 温度灵敏度 (LSB/°C)
 *   36.53 = 0 LSB 对应的温度偏移 (°C)
 *
 * 注: 此温度为芯片内部温度, 非环境温度, 通常比环境高 2~5°C
 */
#define TEMP_SCALE_FACTOR       (1.0f / 340.0f)
#define TEMP_OFFSET             36.53f

/* ==================== 全局调试变量 ==================== */

uint8_t g_mpu6050_i2c_error = 0;   /* 任何 I2C 失败时置 1, 外部读取后手动清零 */
uint8_t g_mpu6050_debug = 0;       /* HAL 错误类型: 1=ERROR, 2=BUSY, 3=TIMEOUT */
uint8_t g_mpu6050_whoami = 0;      /* WHO_AM_I 最后一次读取的原始值 */

/* ==================== I2C 底层读写 ==================== */

/**
 * @brief 向指定寄存器写入 1 字节
 *
 * I2C 帧格式:
 *   [START] [Addr+W] [ACK] [RegAddr] [ACK] [Data] [ACK] [STOP]
 *
 * HAL_I2C_Mem_Write 自动处理上述序列, RegAddr 长度由 I2C_MEMADD_SIZE_8BIT 指定
 *
 * @param reg  寄存器地址 (8-bit)
 * @param data 要写入的值
 * @return 0=成功, -1=失败 (同时设置 g_mpu6050_i2c_error 和 g_mpu6050_debug)
 */
static int MPU6050_WriteReg(uint8_t reg, uint8_t data) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, reg,
                                                  I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    if (status != HAL_OK) {
        g_mpu6050_i2c_error = 1;
        g_mpu6050_debug = (status == HAL_ERROR) ? 1 :
                          (status == HAL_BUSY)  ? 2 : 3;
        return -1;
    }
    return 0;
}

/**
 * @brief 从指定寄存器连续读取 len 字节
 *
 * I2C 帧格式:
 *   [START] [Addr+W] [ACK] [RegAddr] [ACK] [RESTART] [Addr+R] [ACK] [Data...] [NAK] [STOP]
 *
 * 使用 RESTART (重复起始条件) 而非 STOP+START, 保证总线不释放。
 *
 * @param reg  起始寄存器地址
 * @param data 输出缓冲区 (调用方分配, 至少 len 字节)
 * @param len  读取字节数
 * @return 0=成功, -1=失败 (同时设置 g_mpu6050_i2c_error 和 g_mpu6050_debug)
 */
static int MPU6050_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR << 1, reg,
                                                 I2C_MEMADD_SIZE_8BIT, data, len, 100);
    if (status != HAL_OK) {
        g_mpu6050_i2c_error = 1;
        g_mpu6050_debug = (status == HAL_ERROR) ? 1 :
                          (status == HAL_BUSY)  ? 2 : 3;
        return -1;
    }
    return 0;
}

/* ==================== 器件初始化 ==================== */

/**
 * @brief 初始化 MPU6500/6050
 *
 * 配置序列 (严格按此顺序):
 *
 * 1. WHO_AM_I 检测
 *    读 0x75 寄存器, 接受 0x68 (MPU6050) 或 0x70 (MPU6500)
 *    失败 → 返回 -1, 不继续配置
 *
 * 2. 唤醒芯片
 *    写 0x00 → PWR_MGMT_1 (0x6B)
 *    清除 SLEEP 位 (bit6=0), 选择内部 20MHz 时钟源 (CLKSEL=000)
 *    唤醒后等待 10ms 使时钟稳定
 *
 * 3. 采样率分频
 *    写 0x04 → SMPLRT_DIV (0x19)
 *    采样率 = 内部采样率 / (1 + DIV)
 *           = 1kHz / (1 + 4)
 *           = 200Hz
 *    (陀螺仪内部采样率固定为 1kHz, 加速度计固定为 1kHz)
 *
 * 4. 数字低通滤波器 (DLPF)
 *    写 0x03 → CONFIG (0x1A)
 *    DLPF_CFG=3 → 加速度带宽 44Hz, 陀螺仪带宽 42Hz
 *    低通滤波消除高频噪声, 带宽越低噪声越小但响应越慢
 *    DLPF_CFG 可选值: 0(260Hz) ~ 6(5Hz)
 *
 * 5. 陀螺仪量程
 *    写 0x00 → GYRO_CONFIG (0x1B)
 *    FS_SEL=00 → ±250 °/s (灵敏度 131 LSB/°/s)
 *
 * 6. 加速度计量程
 *    写 0x00 → ACCEL_CONFIG (0x1C)
 *    AFS_SEL=00 → ±2g (灵敏度 16384 LSB/g)
 *
 * @return 0=成功, -1=器件不存在或 I2C 错误
 */
int MPU6050_Init(void) {
    g_mpu6050_i2c_error = 0;

    /* ① 检测器件 */
    uint8_t whoami = MPU6050_WhoAmI();
    if (whoami != 0x68 && whoami != 0x70) {
        return -1;  /* 器件不存在或不支持的型号 */
    }

    /* ② 唤醒: 清除 SLEEP 位, 选择内部振荡器 */
    if (MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00) != 0) return -1;
    HAL_Delay(10);  /* 时钟稳定等待 */

    /* ③ 采样率: 1kHz / (1+4) = 200Hz */
    if (MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x04) != 0) return -1;

    /* ④ DLPF: 低通滤波 42Hz (抗混叠+去噪) */
    if (MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x03) != 0) return -1;

    /* ⑤ 陀螺仪量程: ±250 °/s */
    if (MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x00) != 0) return -1;

    /* ⑥ 加速度计量程: ±2g */
    if (MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00) != 0) return -1;

    return 0;
}

/* ==================== 连接检测 ==================== */

/**
 * @brief 检测 MPU6500/6050 是否在线
 *
 * 仅读 WHO_AM_I, 不接受 0x70 (与 Init 不同, 此处仅识别 0x68)
 * 用于运行时快速检测器件是否断开
 *
 * @return 0=在线, -1=离线或型号不匹配
 */
int MPU6050_CheckConnection(void) {
    uint8_t whoami = MPU6050_WhoAmI();
    if (whoami != 0x68) {
        return -1;
    }
    return 0;
}

/* ==================== 读取 WHO_AM_I ==================== */

/**
 * @brief 读取 WHO_AM_I 寄存器 (0x75)
 *
 * 一边读一边更新全局变量 g_mpu6050_whoami (调试用)
 *
 * @return WHO_AM_I 值: 0x68=MPU6050, 0x70=MPU6500
 *         I2C 失败时返回 0x00
 */
uint8_t MPU6050_WhoAmI(void) {
    uint8_t data = 0;
    if (MPU6050_ReadRegs(MPU6050_REG_WHO_AM_I, &data, 1) != 0) {
        g_mpu6050_whoami = 0x00;
        return 0x00;  /* I2C 错误: 返回无效值 */
    }
    g_mpu6050_whoami = data;
    return data;
}

/* ==================== 传感器数据读取 ==================== */

/**
 * @brief 读取三轴加速度
 *
 * 操作:
 *   从 ACCEL_XOUT_H (0x3B) 起连续读 6 字节 → X_H, X_L, Y_H, Y_L, Z_H, Z_L
 *   拼合为有符号 int16 → 乘以换算因子转换为 m/s²
 *
 * 数据格式 (大端):
 *   accel_x = (int16)((X_H << 8) | X_L)
 *   accel_y = (int16)((Y_H << 8) | Y_L)
 *   accel_z = (int16)((Z_H << 8) | Z_L)
 *
 * 静止时典型值:
 *   水平放置: ax≈0, ay≈0, az≈+9.81 m/s² (重力在 Z 轴)
 *   自由落体: ax≈0, ay≈0, az≈0
 *
 * @param accel 输出: 加速度值 (m/s²)
 * @return 0=成功, -1=I2C 错误
 */
int MPU6050_ReadAccel(MPU6050_Accel_t *accel) {
    uint8_t raw[6];
    if (MPU6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, raw, 6) != 0) {
        return -1;
    }

    /* 大端拼合: 高字节在前 */
    int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);

    /* 转换为物理单位 */
    accel->ax = ax_raw * ACCEL_SCALE_FACTOR;
    accel->ay = ay_raw * ACCEL_SCALE_FACTOR;
    accel->az = az_raw * ACCEL_SCALE_FACTOR;
    return 0;
}

/**
 * @brief 读取三轴陀螺仪
 *
 * 操作同 ReadAccel, 从 GYRO_XOUT_H (0x43) 起读 6 字节
 *
 * 静止时典型值: gx≈0, gy≈0, gz≈0 (三轴均接近零)
 * 正方向: 右手定则 (拇指指向轴正方向, 四指弯曲方向为正旋转方向)
 *
 * @param gyro 输出: 角速度 (°/s)
 * @return 0=成功, -1=I2C 错误
 */
int MPU6050_ReadGyro(MPU6050_Gyro_t *gyro) {
    uint8_t raw[6];
    if (MPU6050_ReadRegs(MPU6050_REG_GYRO_XOUT_H, raw, 6) != 0) {
        return -1;
    }

    int16_t gx_raw = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t gy_raw = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t gz_raw = (int16_t)((raw[4] << 8) | raw[5]);

    gyro->gx = gx_raw * GYRO_SCALE_FACTOR;
    gyro->gy = gy_raw * GYRO_SCALE_FACTOR;
    gyro->gz = gz_raw * GYRO_SCALE_FACTOR;
    return 0;
}

/**
 * @brief 读取芯片温度
 *
 * 从 TEMP_OUT_H (0x41) 起读 2 字节 → 拼合为 int16
 * 温度公式 (数据手册):
 *   T(°C) = TEMP_OUT / 340.0 + 36.53
 *
 * 注意事项:
 *   - 这是芯片内部温度传感器的读数, 不是环境温度
 *   - 芯片自发热会导致读数比环境高 2~5°C
 *   - 可用于检测芯片是否正常工作, 不适合替代独立温度传感器
 *
 * @return 温度 (°C), I2C 失败时返回 -273.0 (接近绝对零度, 作为哨兵值)
 */
float MPU6050_ReadTemp(void) {
    uint8_t raw[2];
    if (MPU6050_ReadRegs(MPU6050_REG_TEMP_OUT_H, raw, 2) != 0) {
        return -273.0f;  /* 哨兵值: 正常的芯片温度不可能低于 -40°C */
    }

    int16_t temp_raw = (int16_t)((raw[0] << 8) | raw[1]);
    return (float)temp_raw * TEMP_SCALE_FACTOR + TEMP_OFFSET;
}

/* ==================== 姿态角计算 ==================== */

/**
 * @brief 由加速度数据计算俯仰角和横滚角
 *
 * 原理:
 *   静止状态下, 加速度计测量的是重力矢量 g 在三个轴上的分量
 *   pitch (俯仰) = arctan(ay / sqrt(ax² + az²))   -- 绕 X 轴
 *   roll  (横滚) = arctan(-ax / az)               -- 绕 Y 轴
 *
 *   公式来源:
 *     将重力矢量 [ax, ay, az] 旋转到 [0, 0, g]:
 *     pitch = atan2(ay, sqrt(ax² + az²)) — ay 与 XZ 平面投影的夹角
 *     roll  = atan2(-ax, az)              — ax 与 Z 轴的夹角 (取负)
 *
 *   57.29578 = 180/π, 弧度转换为度
 *
 * 使用前提 (重要):
 *   - 传感器必须处于静止或匀速运动状态
 *   - 加速/减速/振动时, 加速度计测量的不是纯重力, 角度会严重失真
 *   - 如需运动中测姿态, 需要融合陀螺仪数据 (如互补滤波或卡尔曼滤波)
 *
 * @param accel 输入: 加速度数据 (m/s²)
 * @param angle 输出: pitch 和 roll (°)
 */
void MPU6050_CalcAngle(MPU6050_Accel_t *accel, MPU6050_Angle_t *angle) {
    /*
     * pitch: atan2(ay, hypot(ax, az))
     *   物理含义: 绕 X 轴旋转的角度
     *   正值 = 模块前倾 (Y 轴指向前方)
     *   范围: -90° ~ +90°
     */
    angle->pitch = atan2f(accel->ay,
                          sqrtf(accel->ax * accel->ax + accel->az * accel->az))
                   * 57.29578f;

    /*
     * roll: atan2(-ax, az)
     *   物理含义: 绕 Y 轴旋转的角度
     *   正值 = 模块右倾
     *   范围: -180° ~ +180°
     */
    angle->roll  = atan2f(-accel->ax, accel->az) * 57.29578f;
}
