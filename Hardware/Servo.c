#include "stm32f10x.h"                  // Device header
#include "PWM.h"

/**
  * 函    数：舵机初始化
  * 参    数：无
  * 返 回 值：无
  */
void Servo_Init(void)
{
	PWM_Init();									//初始化舵机的底层PWM
}

/**
  * 函    数：舵机1设置角度（PA0-TIM2_CH1）
  * 参    数：Angle 要设置的舵机角度，范围：0~270（270度舵机）
  * 返 回 值：无
  */
void Servo1_SetAngle(float Angle)
{
	PWM_SetCompare1(Angle / 270 * 2000 + 500);	//设置占空比
												//将角度线性变换，对应到舵机要求的占空比范围上
}

/**
  * 函    数：舵机2设置转速（PA1-TIM2_CH2）
  * 参    数：Speed 要设置的舵机转速，范围：-100~100
  *           负值：正转；0：停止；正值：反转
  * 返 回 值：无
  * 说    明：360度连续旋转舵机通过PWM脉宽控制转速和方向
  *           500us ~ 1500us 为一个方向（速度递增）
  *           1500us 为停止
  *           1500us ~ 2500us 为另一个方向（速度递增）
  */
void Servo2_SetSpeed(float Speed)
{
	PWM_SetCompare2(Speed / 100 * 1000 + 1500);	//设置占空比
												//将速度线性变换，对应到舵机要求的占空比范围上
												//-100 -> 500, 0 -> 1500, 100 -> 2500
}
