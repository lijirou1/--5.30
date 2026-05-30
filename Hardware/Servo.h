#ifndef __SERVO_H
#define __SERVO_H

void Servo_Init(void);
void Servo1_SetAngle(float Angle);		//设置舵机1 (PA0) 角度，范围0~270（270度舵机）
void Servo2_SetSpeed(float Speed);		//设置舵机2 (PA1) 转速，范围-100~100（360度连续旋转舵机，0为停止）

#endif
