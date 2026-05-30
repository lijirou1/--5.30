#include "stm32f10x.h"                  // Device header
#include "Delay.h"
/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 开启时钟：GPIOA 和 GPIOB 都属于 APB2 外设，必须同时开启时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA , ENABLE);

	/* ===================== KEY3、KEY4：PB0、PB1（上拉输入） ===================== */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* ===================== KEY1、KEY2：PA11、PA12（上拉输入） ===================== */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * 函    数：按键获取键码
  * 参    数：无
  * 返 回 值：按下按键的键码值（1~4），返回0代表没有按键按下
  *           KEY1(PA11)=1, KEY2(PA12)=2, KEY3(PB0)=3, KEY4(PB1)=4
  * 注意事项：此函数是阻塞式操作，当按键按住不放时，函数会卡住，直到按键松手
  */
uint8_t  Key_GetNum(void)
{
	uint8_t Keynum=0;
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==0)
	{
		Delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==0)
		Delay_ms(20);
		Keynum=3;
	}
	else if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==0)
	{
		Delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==0)
		Delay_ms(20);
		Keynum=4;
	}	
	else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==0)
	{
		Delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==0)
		Delay_ms(20);
		Keynum=1;
	}
	else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)==0)
	{
		Delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)==0)
		Delay_ms(20);
		Keynum=2;
	}	
	return Keynum;
}
