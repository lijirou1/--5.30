/**
 * @file    timer.c
 * @brief   TIM3 定时器模块：20ms 控制节拍
 */

#include "timer.h"

/* ==================== 全局变量 ==================== */

// 20ms 控制节拍标志（在 TIM3 中断中置1，主循环读取并清零）
volatile uint8_t control_flag = 0;

/* ============================================================
 *  TIM3 初始化：20ms中断
 * ============================================================ */
void TIM3_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* 定时周期 = (Prescaler + 1)*(Period + 1) / 72MHz = 20ms */
    TIM_TimeBaseStructure.TIM_Period        = 199;
    TIM_TimeBaseStructure.TIM_Prescaler     = 7199;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ============================================================
 *  TIM3 中断服务函数：每20ms置位 control_flag
 * ============================================================ */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        control_flag = 1;
    }
}