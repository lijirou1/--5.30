#include "stm32f10x.h"

/* 系统滴答计数器（SysTick 中断每 1ms 递增一次） */
static volatile uint32_t Tick = 0;

/**
  * @brief  滴答计数器递增（由 SysTick_Handler 调用）
  * @param  无
  * @retval 无
  */
void IncTick(void)
{
	Tick++;
}

/**
  * @brief  初始化 SysTick 为 1ms 中断模式（非阻塞延时基础）
  * @param  无
  * @retval 无
  * @note   必须在主函数开始时调用一次，之后可通过 GetTick() 获取时间戳
  */
void Delay_Init(void)
{
	/* 配置 SysTick 每 1ms 产生一次中断 */
	/* SystemCoreClock / 1000 = 72,000,000 / 1000 = 72,000 */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* 配置失败则死循环 */
		while (1);
	}
}

/**
  * @brief  获取当前系统滴答计数（毫秒）
  * @param  无
  * @retval 当前毫秒计数值（上电至今的毫秒数，约 49.7 天溢出回零）
  * @note   非阻塞延时用法：
  *         uint32_t start = GetTick();
  *         while (GetTick() - start < 100);   // 仍会阻塞
  *         // 或在大循环中做超时判断：
  *         if (GetTick() - lastTime >= interval) { ... }
  */
uint32_t GetTick(void)
{
	uint32_t tick;
	__disable_irq();		// 关中断保证原子读取
	tick = Tick;
	__enable_irq();			// 恢复中断
	return tick;
}

/* ==================== 保留原有阻塞延时函数 ==================== */

/**
  * @brief  微秒级延时（阻塞，基于正在运行的 SysTick 计数器，不干涉中断）
  * @param  xus 延时时长，范围：0~59652322（理论最大）
  * @retval 无
  * @note   利用 SysTick 已有的 72MHz 递减计数测量时间，不修改任何 SysTick
  *         寄存器，因此不干扰 SysTick 中断和 GetTick() 计数。
  *         全程中断安全，支持中断嵌套。
  */
void Delay_us(uint32_t xus)
{
	uint32_t ticks_needed = 72 * xus;		// 72MHz = 1 tick = 1/72 us
	uint32_t reload = SysTick->LOAD;		// 取当前重装载值（通常为71999）
	uint32_t last_val = SysTick->VAL;		// 当前计数器值
	uint32_t elapsed = 0;

	while (elapsed < ticks_needed)
	{
		uint32_t cur_val = SysTick->VAL;	// 读取最新计数值
		uint32_t delta;

		if (cur_val <= last_val)
		{
			/* 正常递减：直接做差 */
			delta = last_val - cur_val;
		}
		else
		{
			/* 发生了回绕（计数器从0重装载到LOAD），处理溢出 */
			delta = last_val + (reload - cur_val) + 1;
		}

		elapsed += delta;
		last_val = cur_val;
	}
}

/**
  * @brief  毫秒级延时（仍为阻塞，但基于 GetTick 实现，不干扰 SysTick 中断）
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	uint32_t start = GetTick();
	while (GetTick() - start < xms);
}
 
/**
  * @brief  秒级延时（仍为阻塞，但基于 GetTick 实现，不干扰 SysTick 中断）
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	uint32_t start = GetTick();
	while (GetTick() - start < xs * 1000);
} 
