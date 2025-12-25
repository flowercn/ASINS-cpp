#pragma once
#include "stm32f10x.h"

extern "C" {
void Delay_ticks(uint32_t ticks);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);
} // extern "C"
