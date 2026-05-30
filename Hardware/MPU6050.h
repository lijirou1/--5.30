#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count);

uint8_t MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_Kalman_Init(float Q, float R);
void MPU6050_CalibrateGyro(void);			/* 三轴陀螺仪零偏校准（推荐） */
void MPU6050_CalibrateGyroZ(void);			/* 仅Z轴校准（兼容旧代码） */
void MPU6050_UpdateYaw_Filtered(int16_t GZ);
void MPU6050_UpdateYaw(int16_t GZ);
float MPU6050_GetYaw(void);
int16_t MPU6050_GetGyroZBias(void);

/* 卡尔曼 + 互补滤波角度计算 */
void MPU6050_UpdateAngles(void);        /* 更新俯仰/横滚角（需每20ms调用） */
float MPU6050_GetPitch(void);           /* 获取俯仰角（度） */
float MPU6050_GetRoll(void);            /* 获取横滚角（度） */
void MPU6050_ResetAngles(void);         /* 复位滤波器状态 */

#endif
