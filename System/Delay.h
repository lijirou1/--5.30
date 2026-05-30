#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

/* 非阻塞延时（基于 SysTick 中断） */
void Delay_Init(void);			// 初始化 SysTick 定时器（1ms 中断）
uint32_t GetTick(void);			// 获取当前系统滴答计数（毫秒）

/* 保留原有的阻塞延时函数（向后兼容） */
void Delay_us(uint32_t us);		// 基于SysTick计数器（不干涉中断，完全中断安全）
void Delay_ms(uint32_t ms);		// 基于GetTick()（中断安全）
void Delay_s(uint32_t s);		// 基于GetTick()（中断安全）

#endif
