/**
 * @file Delay.cpp
 * @brief 微秒、毫秒级延时函数实现
 * @details 使用Cortex-M3内核的SysTick定时器实现精确的延时功能。
 * @author Gemini
 * @date 2025-08-30
 */

#include "Delay.h"

//==================================================================================================
// 公共函数实现 (Public Function Implementations)
//==================================================================================================

/**
 * @brief CPU时钟周期延时
 * @details 直接操作SysTick定时器，实现基于CPU时钟周期的最精确延时。
 *          假设系统时钟 (HCLK) 为 72MHz。
 * @param[in] ticks 要延时的CPU时钟周期数。最大值约为16.7M (2^24)。
 */
void Delay_ticks(uint32_t ticks)
{
    // 1. 设置SysTick重装载值
    SysTick->LOAD = ticks;
    // 2. 清空当前计数值
    SysTick->VAL = 0x00;
    // 3. 设置时钟源为HCLK (72MHz), 使能SysTick定时器
    SysTick->CTRL = 0x00000005;
    // 4. 等待计数到0 (COUNTFLAG位被置1)
    while(!(SysTick->CTRL & 0x00010000));
    // 5. 关闭SysTick定时器
    SysTick->CTRL = 0x00000004;
}

/**
 * @brief 微秒级延时
 * @details 封装了Delay_ticks函数，将微秒转换为CPU时钟周期。
 *          假设系统时钟 (HCLK) 为 72MHz, 1us = 72 ticks。
 * @param[in] us 要延时的微秒数。实际最大延时约为 233015 us。
 */
void Delay_us(uint32_t us)
{
	SysTick->LOAD = 72 * us;
	SysTick->VAL = 0x00;
	SysTick->CTRL = 0x00000005;
	while(!(SysTick->CTRL & 0x00010000));
	SysTick->CTRL = 0x00000004;
}

/**
 * @brief 毫秒级延时
 * @details 通过循环调用Delay_us(1000)实现。
 * @param[in] ms 要延时的毫秒数。
 */
void Delay_ms(uint32_t ms)
{
	while(ms--)
	{
		Delay_us(1000);
	}
}
 
/**
 * @brief 秒级延时
 * @details 通过循环调用Delay_ms(1000)实现。
 * @param[in] s 要延时的秒数。
 */
void Delay_s(uint32_t s)
{
	while(s--)
	{
		Delay_ms(1000);
	}
}
