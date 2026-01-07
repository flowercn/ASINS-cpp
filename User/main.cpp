/**************************************************************
@Project:Modern C++ High-Performance Full-Stack Telemetry Architecture
@Author: Dengyihang
@Date: 2026/01/04
***************************************************************/
#include "stm32f10x.h"
#include "GpioPin.h" 
#include "Serial.h"
#include "ICM20602.h"
#include "Timer.h"

#include "PlatformSTM32.h" // 适配器
#include "ImuService.h"    // 核心业务
using LedRed   = PA<0>;
using LedGreen = PA<1>;
//    - 创建适配器 (连接硬件)
//    - 创建服务 (注入适配器)
Stm32IoPort g_platformIo;
ImuService  g_imuService(g_platformIo);

int main(void) {
    LedRed::initOutputPP();
    LedGreen::initOutputPP();
    LedRed::reset(); // 
    LedGreen::set();

	HwTimer_Init();
    Serial.init(921600);
    ImuSensors.init(); 

    static SensorFrame currentFrame; 
    while (1) {
        FetchSensorData_STM32(currentFrame);
        g_imuService.process(currentFrame);
    }
}
