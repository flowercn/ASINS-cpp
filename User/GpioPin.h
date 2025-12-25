/**
 * @file GpioPin.h
 * @brief C++17 模板元编程 GPIO 驱动
 * @details 使用模板参数在编译期确定端口和引脚，实现零运行时开销的 GPIO 操作。
 *          编译器会直接将这些代码展开为寄存器操作指令。
 */

#pragma once

#include "stm32f10x.h"

// ========================================
// GPIO 模板驱动类
// ========================================

template<uint32_t PortAddr, uint16_t PinNumber>
class GpioPin {
public:
    static constexpr uint32_t BaseAddr = PortAddr;
    static constexpr uint16_t PinMask = (1 << PinNumber);
    
    // 编译期常量：端口指针
    static inline GPIO_TypeDef* Port() {
        return reinterpret_cast<GPIO_TypeDef*>(PortAddr);
    }

    // ========================================
    // 操作函数 (全部强制内联)
    // ========================================

    // 置高电平 (BSRR 寄存器)
    static inline void set() {
        Port()->BSRR = PinMask;
    }

    // 置低电平 (BRR 寄存器)
    static inline void reset() {
        Port()->BRR = PinMask;
    }

    // 翻转电平 (读 ODR 并异或)
    static inline void toggle() {
        Port()->ODR ^= PinMask;
    }

    // 读取输入电平
    static inline bool read() {
        return (Port()->IDR & PinMask) != 0;
    }

    // 初始化为输出模式
    static void initOutput() {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = PinMask;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(Port(), &GPIO_InitStructure);
    }
};

// ========================================
// 端口别名模板
// ========================================

template<uint16_t N> using PA = GpioPin<GPIOA_BASE, N>;
template<uint16_t N> using PB = GpioPin<GPIOB_BASE, N>;
template<uint16_t N> using PC = GpioPin<GPIOC_BASE, N>;
template<uint16_t N> using PD = GpioPin<GPIOD_BASE, N>;
template<uint16_t N> using PE = GpioPin<GPIOE_BASE, N>;
template<uint16_t N> using PF = GpioPin<GPIOF_BASE, N>;
template<uint16_t N> using PG = GpioPin<GPIOG_BASE, N>;
