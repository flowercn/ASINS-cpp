// Timer.h
#pragma once
#include "stm32f10x.h"

// 变量声明 (extern)
extern volatile uint32_t g_hwTimeHigh;

// 函数声明
void HwTimer_Init();

// 获取时间戳 (为了性能保留 inline，这样调用处直接展开，最快)
inline uint32_t GetHwTimestamp() {
    // 关中断保护 (可选，看你是否介意极其罕见的读取竞争，不加也行)
    return (g_hwTimeHigh << 16) | TIM2->CNT;
}