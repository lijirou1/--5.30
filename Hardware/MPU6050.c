#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "MyI2C.h"
#include "delay.h"
#include <math.h>

#define MPU6050_ADDRESS        0xD0
#define MPU6050_ID             0x68

/* 量程配置 */
#define GYRO_RANGE_CFG         0x08        /* ±500°/s */
#define ACCEL_RANGE_CFG        0x18        /* ±8g */
#define GYRO_SENSITIVITY      98.0f        /* ±500°/s → LSB/(°/s) */
#define ACCEL_SENSITIVITY      4096.0f     /* ±8g → LSB/g */
#define PI_F                   3.14159265f
#define RAD2DEG                57.29578f   /* 180/π */

/* 采样周期（必须与TIM4中断周期一致） */
#define DT                     0.02f       /* 50Hz = 20ms */

/* 互补滤波系数（α越大，陀螺仪权重越高） */
#define CF_ALPHA               0.96f

/* ===================== 静态变量 ===================== */

/* ---- 偏航（Z轴陀螺仪 + 卡尔曼 + ZUPT，已有） ---- */
static int16_t gyro_z_bias = 0;
static int16_t gyro_x_bias = 0;        /* X轴陀螺仪零偏（用于横滚） */
static int16_t gyro_y_bias = 0;        /* Y轴陀螺仪零偏（用于俯仰） */
static float Yaw = 0.0f;
uint8_t imu_init_ok = 0;

/* ---- 卡尔曼（原Yaw专用，保留兼容） ---- */
static float kalman_x = 0.0f;
static float kalman_P = 0.0f;
static float kalman_Q = 0.02f;
static float kalman_R = 0.5f;
static uint8_t kalman_first = 1;

/* ---- 俯仰/横滚：互补滤波状态 ---- */
static float pitch_cf = 0.0f;       /* 互补滤波后的俯仰角（度） */
static float roll_cf = 0.0f;        /* 互补滤波后的横滚角（度） */

/* ---- 俯仰：独立卡尔曼滤波器 ---- */
static float kf_pitch_x = 0.0f;
static float kf_pitch_P = 1.0f;
static float kf_pitch_Q = 0.01f;
static float kf_pitch_R = 0.3f;
static uint8_t kf_pitch_first = 1;

/* ---- 横滚：独立卡尔曼滤波器 ---- */
static float kf_roll_x = 0.0f;
static float kf_roll_P = 1.0f;
static float kf_roll_Q = 0.01f;
static float kf_roll_R = 0.3f;
static uint8_t kf_roll_first = 1;

/* ===================== I2C基本操作 ===================== */

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(Data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    MyI2C_ReceiveAck();
    Data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();

    return Data;
}

void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
    uint8_t i;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    MyI2C_ReceiveAck();
    for (i = 0; i < Count; i++)
    {
        DataArray[i] = MyI2C_ReceiveByte();
        MyI2C_SendAck(i < Count - 1 ? 0 : 1);
    }
    MyI2C_Stop();
}

/* ===================== 初始化 & ID ===================== */

uint8_t MPU6050_Init(void)
{
    uint16_t timeout = 0;

    MyI2C_Init();
    Delay_ms(20);

    while (MPU6050_GetID() != MPU6050_ID)
    {
        Delay_ms(1);
        if (++timeout > 255)
        {
            imu_init_ok = 0;
            return 1;
        }
    }

    /* 配置MPU6050 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);        /* 时钟源：X轴陀螺仪 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);        /* 所有轴使能 */
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);        /* 采样率 = 1kHz / (1+9) = 100Hz */
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);             /* DLPF带宽 = 5Hz */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, GYRO_RANGE_CFG);   /* ±500°/s */
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, ACCEL_RANGE_CFG); /* ±8g */

    /* 复位滤波器状态 */
    MPU6050_ResetAngles();

    imu_init_ok = 1;
    return 0;
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/* ===================== 原始数据读取 ===================== */

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t Data[14];

    MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14);

    *AccX = (int16_t)((uint16_t)Data[0] << 8 | Data[1]);
    *AccY = (int16_t)((uint16_t)Data[2] << 8 | Data[3]);
    *AccZ = (int16_t)((uint16_t)Data[4] << 8 | Data[5]);

    *GyroX = (int16_t)((uint16_t)Data[8] << 8 | Data[9]);
    *GyroY = (int16_t)((uint16_t)Data[10] << 8 | Data[11]);
    *GyroZ = (int16_t)((uint16_t)Data[12] << 8 | Data[13]);
}

/* ===================== 1D卡尔曼滤波器 ===================== */

/* 通用1D卡尔曼初始化 */
static void Kalman1D_Init(float *x, float *P, float Q, float R, uint8_t *first)
{
    *x = 0.0f;
    *P = 1.0f;
    *first = 1;
}

/* 通用1D卡尔曼更新：返回滤波后的值 */
static float Kalman1D_Update(float measurement, float *x, float *P,
                              float Q, float R, uint8_t *first)
{
    float K;

    if (*first)
    {
        *x = measurement;
        *first = 0;
        return *x;
    }

    /* 预测：状态预测 */
    *P += Q;

    /* 更新：卡尔曼增益 */
    K = *P / (*P + R);

    /* 更新：状态修正 */
    *x += K * (measurement - *x);

    /* 更新：误差协方差 */
    *P *= (1.0f - K);

    return *x;
}

/* ===================== 卡尔曼（原Yaw专用接口，保留兼容） ===================== */

void MPU6050_Kalman_Init(float Q, float R)
{
    kalman_Q = Q;
    kalman_R = R;
    kalman_x = 0.0f;
    kalman_P = 1.0f;
    kalman_first = 1;
}

static void Kalman_SetAdaptiveQ(float raw_dps)
{
    float abs_dps = raw_dps;
    if (abs_dps < 0.0f) abs_dps = -abs_dps;

    if (abs_dps < 5.0f)
        kalman_Q = 0.02f;
    else if (abs_dps < 30.0f)
        kalman_Q = 0.1f;
    else
        kalman_Q = 0.5f;
}

static float Kalman_Update(float measurement)
{
    return Kalman1D_Update(measurement, &kalman_x, &kalman_P,
                           kalman_Q, kalman_R, &kalman_first);
}

/* ===================== 三轴陀螺仪校准 ===================== */

/**
  * 函    数：校准三轴陀螺仪零偏
  * 参    数：无
  * 返 回 值：无
  * 说    明：采集100次取平均，得到X/Y/Z三轴零偏
  *           必须在传感器静止时调用
  */
void MPU6050_CalibrateGyro(void)
{
    int i;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int16_t GX, GY, GZ;
    int16_t AX, AY, AZ;

    /* 清零，避免校准前残留数据干扰 */
    gyro_x_bias = 0;
    gyro_y_bias = 0;
    gyro_z_bias = 0;

    for (i = 0; i < 100; i++)
    {
        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
        sum_gx += GX;
        sum_gy += GY;
        sum_gz += GZ;
        Delay_ms(2);
    }

    gyro_x_bias = (int16_t)(sum_gx / 100);
    gyro_y_bias = (int16_t)(sum_gy / 100);
    gyro_z_bias = (int16_t)(sum_gz / 100);

    MPU6050_Kalman_Init(0.02f, 0.5f);

    /* 互补/卡尔曼滤波器状态复位，以当前加速度计角度为初值 */
    MPU6050_ResetAngles();
}

/* 保留旧接口名，方便原有代码调用 */
void MPU6050_CalibrateGyroZ(void)
{
    MPU6050_CalibrateGyro();
}

/* ===================== 偏航角（Yaw）：卡尔曼+ZUPT ===================== */

#define ZUPT_THRESHOLD_DPS     0.6f
#define ZUPT_COUNT_THRESHOLD   5
static uint8_t zupt_count = 0;
static float bias_correction = 0.0f;
static uint8_t was_stationary = 0;

#define TURN_HIGH_SPEED_THRESHOLD  30.0f
#define ZUPT_COOLDOWN_CYCLES       25
static uint8_t turn_cooldown = 0;

void MPU6050_UpdateYaw_Filtered(int16_t GZ)
{
    float raw_dps;
    float filtered_dps;
    float abs_raw_dps;

    raw_dps = (float)(GZ - gyro_z_bias) / GYRO_SENSITIVITY;

    Kalman_SetAdaptiveQ(raw_dps);
    filtered_dps = Kalman_Update(raw_dps);

    abs_raw_dps = raw_dps;
    if (abs_raw_dps < 0.0f) abs_raw_dps = -abs_raw_dps;

    if (abs_raw_dps >= TURN_HIGH_SPEED_THRESHOLD)
    {
        turn_cooldown = ZUPT_COOLDOWN_CYCLES;
    }
    else if (turn_cooldown > 0)
    {
        turn_cooldown--;
    }

    if (turn_cooldown == 0)
    {
        if (filtered_dps > -ZUPT_THRESHOLD_DPS && filtered_dps < ZUPT_THRESHOLD_DPS)
        {
            if (zupt_count < ZUPT_COUNT_THRESHOLD)
                zupt_count++;

            if (zupt_count >= ZUPT_COUNT_THRESHOLD)
            {
                was_stationary = 1;

                bias_correction += filtered_dps * 0.1f;
                if (bias_correction > 0.25f)
                {
                    gyro_z_bias += 1;
                    bias_correction = 0.0f;
                }
                else if (bias_correction < -0.25f)
                {
                    gyro_z_bias -= 1;
                    bias_correction = 0.0f;
                }

                return;
            }
        }
        else
        {
            zupt_count = 0;

            if (was_stationary)
            {
                was_stationary = 0;
                bias_correction = 0.0f;
            }
        }
    }
    else
    {
        if (!(filtered_dps > -ZUPT_THRESHOLD_DPS && filtered_dps < ZUPT_THRESHOLD_DPS))
        {
            zupt_count = 0;
        }

        if (was_stationary)
        {
            was_stationary = 0;
            bias_correction = 0.0f;
        }
    }

    Yaw += filtered_dps * DT;

    if (Yaw >= 360.0f) Yaw -= 360.0f;
    if (Yaw < 0.0f) Yaw += 360.0f;
}

void MPU6050_UpdateYaw(int16_t GZ)
{
    Yaw += (float)(GZ - gyro_z_bias) / GYRO_SENSITIVITY * DT;

    if (Yaw >= 360.0f) Yaw -= 360.0f;
    if (Yaw < 0.0f) Yaw += 360.0f;
}

float MPU6050_GetYaw(void)
{
    return Yaw;
}

int16_t MPU6050_GetGyroZBias(void)
{
    return gyro_z_bias;
}

/* ===================== 俯仰/横滚：加速度计角度计算 ===================== */

/**
  * 函    数：从加速度计原始值计算俯仰角和横滚角
  * 参    数：ax, ay, az — 加速度计原始值
  *           pPitch — 输出俯仰角（度）
  *           pRoll  — 输出横滚角（度）
  * 说    明：
  *           俯仰角 = atan2(-ax, sqrt(ay² + az²)) × 180/π
  *           横滚角 = atan2( ay, az ) × 180/π
  *           当Z轴朝上水平放置时，ax=ay=0, az=+1g → pitch=0, roll=0
  *           抬头为正俯仰，右侧下沉为正横滚
  */
static void MPU6050_AccelToAngles(int16_t ax, int16_t ay, int16_t az,
                                   float *pPitch, float *pRoll)
{
    float a_x = (float)ax / ACCEL_SENSITIVITY;   /* 单位：g */
    float a_y = (float)ay / ACCEL_SENSITIVITY;
    float a_z = (float)az / ACCEL_SENSITIVITY;

    /* 俯仰角：绕Y轴旋转 */
    *pPitch = atan2f(-a_x, sqrtf(a_y * a_y + a_z * a_z)) * RAD2DEG;

    /* 横滚角：绕X轴旋转 */
    *pRoll = atan2f(a_y, a_z) * RAD2DEG;
}

/* ===================== 俯仰/横滚：互补滤波器 ===================== */

/**
  * 函    数：互补滤波更新单个角度
  * 参    数：angle_cf — 当前滤波角度（会被更新）
  *           gyro_rate — 对应轴陀螺仪角速度（°/s）
  *           acc_angle — 加速度计计算角度（°）
  * 说    明：
  *           angle = α × (angle + gyro_rate × dt) + (1-α) × acc_angle
  *           陀螺仪积分提供高频响应，加速度计修正低频漂移
  */
static void Complementary_Update(float *angle_cf, float gyro_rate, float acc_angle)
{
    *angle_cf = CF_ALPHA * (*angle_cf + gyro_rate * DT)
              + (1.0f - CF_ALPHA) * acc_angle;
}

/* ===================== 俯仰/横滚：主更新接口 ===================== */

/**
  * 函    数：更新三轴姿态角（需每20ms调用一次）
  * 参    数：无
  * 返 回 值：无
  * 说    明：
  *           1. 一次性读取六轴原始数据（仅一次I2C事务）
  *           2. 更新偏航角（卡尔曼滤波 + ZUPT）
  *           3. 计算加速度计俯仰/横滚
  *           4. 互补滤波融合陀螺仪 + 加速度计
  *           5. 卡尔曼滤波进一步平滑
  */
void MPU6050_UpdateAngles(void)
{
    int16_t ax, ay, az, gx, gy, gz;
    float pitch_acc, roll_acc;
    float gyroX_dps, gyroY_dps;

    /* 1. 一次性读取六轴数据 */
    MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);

    /* 2. 更新偏航角（原Yaw逻辑） */
    MPU6050_UpdateYaw_Filtered(gz);

    /* 3. 加速度计 → 俯仰/横滚角度 */
    MPU6050_AccelToAngles(ax, ay, az, &pitch_acc, &roll_acc);

    /* 4. 陀螺仪 → °/s（减去零偏后转换） */
    gyroX_dps = (float)(gx - gyro_x_bias) / GYRO_SENSITIVITY;   /* X轴角速度 → 横滚速率 */
    gyroY_dps = (float)(gy - gyro_y_bias) / GYRO_SENSITIVITY;    /* Y轴角速度 → 俯仰速率 */

    /* 5. 互补滤波融合
     * 注意：GyroY正方向使机头下降（负俯仰），故取反
     *       GyroX正方向使右侧下沉（正横滚），保持原号
     */
    Complementary_Update(&pitch_cf, -gyroY_dps, pitch_acc);
    Complementary_Update(&roll_cf,   gyroX_dps, roll_acc);

    /* 6. 卡尔曼滤波进一步平滑 */
    Kalman1D_Update(pitch_cf, &kf_pitch_x, &kf_pitch_P,
                    kf_pitch_Q, kf_pitch_R, &kf_pitch_first);
    Kalman1D_Update(roll_cf, &kf_roll_x, &kf_roll_P,
                    kf_roll_Q, kf_roll_R, &kf_roll_first);
}

/* ===================== 俯仰/横滚：取值接口 ===================== */

float MPU6050_GetPitch(void)
{
    return kf_pitch_x;      /* 返回卡尔曼滤波后的俯仰角 */
}

float MPU6050_GetRoll(void)
{
    return kf_roll_x;       /* 返回卡尔曼滤波后的横滚角 */
}

/* ===================== 复位滤波器 ===================== */

/**
  * 函    数：复位所有滤波器状态
  * 参    数：无
  * 返 回 值：无
  * 说    明：以当前加速度计角度为初始值，避免启动时跳变
  */
void MPU6050_ResetAngles(void)
{
    int16_t ax, ay, az, gx, gy, gz;
    float pitch_acc, roll_acc;

    /* 读一次加速度计，获取初始角度 */
    MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
    MPU6050_AccelToAngles(ax, ay, az, &pitch_acc, &roll_acc);

    /* 互补滤波状态 = 加速度计初始角度 */
    pitch_cf = pitch_acc;
    roll_cf  = roll_acc;

    /* 卡尔曼滤波状态 = 加速度计初始角度 */
    Kalman1D_Init(&kf_pitch_x, &kf_pitch_P, kf_pitch_Q, kf_pitch_R, &kf_pitch_first);
    Kalman1D_Init(&kf_roll_x,  &kf_roll_P,  kf_roll_Q,  kf_roll_R,  &kf_roll_first);

    kf_pitch_x = pitch_acc;
    kf_roll_x  = roll_acc;

    /* 偏航不复位（保持绝对参考） */
    /* 偏航卡尔曼也不复位 */
}
