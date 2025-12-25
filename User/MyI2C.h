#pragma once

#include "stm32f10x.h"

// I2C总线数量
#define I2C_NUM 64

// 定义I2C引脚结构体
typedef struct {
    GPIO_TypeDef* GPIOx;
    uint16_t      PINx;
} I2C_PIN;

// 全局错误标志
extern volatile int g_i2c_error_detected;

// 公共 API 函数声明
extern "C" {
    void I2C_Config();
    void I2C_SendByte(uint8_t byte, uint8_t* Ack);
    void I2C_ReceiveByte(uint8_t* byte, uint8_t Ack);
    void I2C_Start();
    void I2C_Stop();
}
