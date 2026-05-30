#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Servo.h"
#include "Key.h"
#include "timer.h"
#include "MPU6050.h"

uint8_t KeyNum = 0;				//定义用于接收键码的变量
float Angle1 = 30.0f;			//定义舵机1的角度变量（30~270）
float Speed2 = 0;				//定义舵机2的转速变量（-100~100）

/**
  * 函    数：陀螺仪实时追踪云台
  * 参    数：无
  * 返 回 值：无
  * 说    明：读取MPU6050滤波后的俯仰角，实时控制舵机跟随姿态
  *           俯仰角（卡尔曼+互补滤波）→ 舵机1角度（30°~270°）
  *           偏航角速度（陀螺仪Z轴）→ 舵机2转速（-100~100）
  *           按Key1退出追踪模式
  */
void Gimbal_Tracking_Mode(void)
{
	float pitch_angle = 0.0f;
	float angle1, speed2;

	OLED_Clear();
	OLED_ShowString(1, 1, "> TRACKING MODE");
	OLED_ShowString(4, 1, "Key1: Exit");
	Delay_ms(800);

	/* 先回到中位 */
	Servo1_SetAngle(150);
	Servo2_SetSpeed(0);
	Delay_ms(200);

	while (1)
	{
		/* 按Key1退出追踪模式 */
		if (Key_GetNum() == 1)
		{
			break;
		}

		/* 20ms控制节拍 */
		if (control_flag == 1)
		{
			static uint8_t yaw_ref_inited = 0;
			static float yaw_ref = 0.0f;
			float current_yaw;
			float yaw_error;
			float roll_angle;

			control_flag = 0;

			/* 更新MPU6050滤波角度（内部自动读取六轴数据，仅一次I2C读取）
			 * 同时更新：俯仰（卡尔曼+互补滤波）、偏航（卡尔曼滤波+ZUPT）
			 */
			MPU6050_UpdateAngles();

			/* ========== 俯仰控制 → 舵机1（角度舵机） ========== */
			/* 获取卡尔曼+互补滤波后的俯仰角 */
			pitch_angle = MPU6050_GetPitch();

			/* 映射到舵机1角度（30°~270°），俯仰正向对应舵机向上补偿 */
			angle1 = 150.0f + pitch_angle * 1.0f;
			if (angle1 < 30.0f)  angle1 = 30.0f;
			if (angle1 > 270.0f) angle1 = 270.0f;
			Servo1_SetAngle(angle1);

			/* ========== 偏航控制 → 舵机2（连续旋转舵机） ========== */
			/* 位置式P控制：以进入追踪模式时的偏航角为参考，
			 * 偏航误差 → P控制器 → 舵机转速。
			 * 传感器装在云台上时，舵机转动会改变偏航读数，
			 * 形成闭环，误差收敛到0后舵机自然停止。
			 */
			current_yaw = MPU6050_GetYaw();

			/* 首次运行：记录当前偏航角为参考零点 */
			if (!yaw_ref_inited)
			{
				yaw_ref = current_yaw;
				yaw_ref_inited = 1;
			}

			/* 计算偏航误差 */
			yaw_error = current_yaw - yaw_ref;

			/* 归一化到 -180°~180° */
			if (yaw_error > 180.0f)  yaw_error -= 360.0f;
			if (yaw_error < -180.0f) yaw_error += 360.0f;

			/* P控制器：转速 = -Kp × 误差（取反补偿旋转方向） */
			speed2 = -yaw_error * 1.0f;

			/* 死区：忽略极小误差，防止零点抖动 */
			if (speed2 > -3.0f && speed2 < 3.0f)
				speed2 = 0.0f;

			if (speed2 > 100.0f)  speed2 = 100.0f;
			if (speed2 < -100.0f) speed2 = -100.0f;
			Servo2_SetSpeed(speed2);

			/* 获取横滚角 */
			roll_angle = MPU6050_GetRoll();

			/* OLED实时显示三轴角度（×10显示一位小数） */
			/* 第1行：俯仰角  P:+12.3 */
			OLED_ShowString(1, 1, "P:");
			OLED_ShowSignedNum(1, 3, (int16_t)(pitch_angle * 10), 3);

			/* 第2行：偏航角  Y:+45.6 */
			OLED_ShowString(2, 1, "Y:");
			OLED_ShowSignedNum(2, 3, (int16_t)(current_yaw * 10), 4);

			/* 第3行：横滚角 + 舵机1角度  R:+12.3 S1:150 */
			OLED_ShowString(3, 1, "R:");
			OLED_ShowSignedNum(3, 3, (int16_t)(roll_angle * 10), 3);
			OLED_ShowString(3, 9, "S1:");
			OLED_ShowNum(3, 12, (uint32_t)angle1, 3);

			/* 第4行：舵机2转速 + 状态  S2:+45 TRACK */
			OLED_ShowString(4, 1, "S2:");
			OLED_ShowSignedNum(4, 4, (int16_t)speed2, 3);
			OLED_ShowString(4, 8, "TRACK");
		}
	}

	/* 退出追踪模式，恢复初始状态 */
	Servo1_SetAngle(150);
	Servo2_SetSpeed(0);
	OLED_Clear();
	OLED_ShowString(1, 1, "S1:");
	OLED_ShowString(2, 1, "S2:");
	OLED_ShowNum(1, 4, 150, 3);
	OLED_ShowSignedNum(2, 4, 0, 3);
}

/**
  * 函    数：陀螺仪超级自检
  * 参    数：无
  * 返 回 值：无
  * 说    明：全面检测MPU6050传感器和舵机系统
  *           步骤1：MPU6050 ID检测
  *           步骤2：陀螺仪Z轴校准（显示零偏值）
  *           步骤3：读取并显示六轴原始数据
  *           步骤4：舵机联动扫描测试
  */
void Gimbal_SuperSelfCheck(void)
{
	uint8_t id;
	int16_t AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
	int16_t gz_bias;
	uint8_t err = 0;
	uint8_t i;

	OLED_Clear();
	OLED_ShowString(1, 1, "=== SUPER CHECK ===");
	Delay_ms(600);

	/* === 步骤1：MPU6050 ID检测 === */
	OLED_Clear();
	OLED_ShowString(1, 1, "1. MPU6050 ID...");
	id = MPU6050_GetID();
	if (id == 0x68)
	{
		OLED_ShowString(2, 1, "ID:0x68 PASS!");
	}
	else
	{
		OLED_ShowString(2, 1, "ID:0x");
		OLED_ShowHexNum(2, 5, id, 2);
		OLED_ShowString(2, 7, " FAIL!");
		err = 1;
	}
	Delay_ms(800);

	/* === 步骤2：三轴陀螺仪校准 === */
	OLED_Clear();
	OLED_ShowString(1, 1, "2. CALIBRATING...");
	OLED_ShowString(2, 1, "Keep Still!");
	MPU6050_CalibrateGyro();
	gz_bias = MPU6050_GetGyroZBias();
	OLED_Clear();
	OLED_ShowString(1, 1, "2. GYRO BIAS OK");
	OLED_ShowSignedNum(2, 1, gz_bias, 6);
	Delay_ms(1000);

	/* === 步骤3：读取并显示各轴数据 === */
	MPU6050_GetData(&AccX, &AccY, &AccZ, &GyroX, &GyroY, &GyroZ);

	OLED_Clear();
	OLED_ShowString(1, 1, "3. ACCELEROMETER");
	OLED_ShowString(2, 1, "AX:");
	OLED_ShowSignedNum(2, 4, AccX, 5);
	OLED_ShowString(3, 1, "AY:");
	OLED_ShowSignedNum(3, 4, AccY, 5);
	OLED_ShowString(4, 1, "AZ:");
	OLED_ShowSignedNum(4, 4, AccZ, 5);
	Delay_ms(1500);

	OLED_Clear();
	OLED_ShowString(1, 1, "3. GYROSCOPE");
	OLED_ShowString(2, 1, "GX:");
	OLED_ShowSignedNum(2, 4, GyroX, 5);
	OLED_ShowString(3, 1, "GY:");
	OLED_ShowSignedNum(3, 4, GyroY, 5);
	OLED_ShowString(4, 1, "GZ:");
	OLED_ShowSignedNum(4, 4, GyroZ, 5);
	Delay_ms(1500);

	/* === 步骤4：舵机联动扫描测试 === */
	OLED_Clear();
	OLED_ShowString(1, 1, "4. SERVO SCAN...");
	Servo2_SetSpeed(30);					//水平慢速正转
	for (i = 3; i <= 27; i++)				//竖直30°~270°扫描
	{
		Servo1_SetAngle(i * 10);
		OLED_ShowString(2, 1, "S1:");
		OLED_ShowNum(2, 4, i * 10, 3);
		Delay_ms(60);
	}
	Servo2_SetSpeed(-30);					//水平慢速反转
	for (i = 27; i >= 3; i--)				//竖直270°~30°回扫
	{
		Servo1_SetAngle(i * 10);
		OLED_ShowString(2, 1, "S1:");
		OLED_ShowNum(2, 4, i * 10, 3);
		Delay_ms(60);
	}
	Servo2_SetSpeed(0);						//水平停止
	Servo1_SetAngle(150);					//竖直回中
	OLED_ShowString(3, 1, "SERVO OK!");
	Delay_ms(500);

	/* === 显示最终结果 === */
	OLED_Clear();
	if (err == 0)
	{
		OLED_ShowString(1, 1, "CHECK: ALL PASS!");
		OLED_ShowString(3, 1, "System Healthy");
	}
	else
	{
		OLED_ShowString(1, 1, "CHECK: FAILED!");
		OLED_ShowString(3, 1, "Check MPU6050");
	}
	Delay_ms(1500);

	/* 恢复初始显示 */
	OLED_Clear();
	OLED_ShowString(1, 1, "S1:");
	OLED_ShowString(2, 1, "S2:");
	Servo1_SetAngle(150);
	Servo2_SetSpeed(0);
	OLED_ShowNum(1, 4, 150, 3);
	OLED_ShowSignedNum(2, 4, 0, 3);
}

int main(void)
{
	/*模块初始化*/
	Delay_Init();		//非阻塞延时初始化（SysTick 1ms中断）
	OLED_Init();		//OLED初始化
	Servo_Init();		//舵机初始化
	Key_Init();			//按键初始化
	TIM3_Init();		//TIM3定时器初始化（20ms中断，用于控制节拍）
	MPU6050_Init();		//MPU6050陀螺仪初始化
	MPU6050_CalibrateGyro();	//三轴陀螺仪零偏校准
	MPU6050_ResetAngles();		//角度滤波器复位
	/*显示静态字符串*/
	OLED_ShowString(1, 1, "S1:");		//舵机1角度
	OLED_ShowString(2, 1, "S2:");		//舵机2转速
	
	/*设置舵机初始位置*/
	Servo1_SetAngle(150);				//舵机1转到150°（中心）
	Servo2_SetSpeed(0);					//舵机2停止
	OLED_ShowNum(1, 4, 150, 3);		//显示初始角度150
	OLED_ShowSignedNum(2, 4, 0, 3);		//显示初始转速0
	
	while (1)
	{
		KeyNum = Key_GetNum();			//获取按键键码
		if (KeyNum == 1)				//按键1按下，陀螺仪实时追踪云台
		{
			Gimbal_Tracking_Mode();
		}
		if (KeyNum == 2)				//按键2按下，陀螺仪超级自检
		{
			Gimbal_SuperSelfCheck();
		}
		if (KeyNum == 3)				//按键3按下，原有的云台一键自检
		{
		}

		/* 使用 TIM3 20ms 控制节拍——OLED第3行心跳指示 */
		if (control_flag == 1)
		{
			static uint8_t tick_cnt = 0;
			static uint8_t blink = 0;

			control_flag = 0;			//清除标志
			tick_cnt++;
			if (tick_cnt % 25 == 0)		//每500ms(25*20ms)切换一次显示
			{
				blink = !blink;
				if (blink)
					OLED_ShowString(3, 1, "RUN  ");
				else
					OLED_ShowString(3, 1, "     ");
			}
		}
	}
}
