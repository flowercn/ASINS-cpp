#pragma once
#include "stm32f10x.h"
#include "ICM20602_Reg.h"
#include "MyI2C.h"

static constexpr uint8_t ICM20602_I2C_ADDRESS = 0xD0;
// 纯粹的硬件原始数据缓存 (12个寄存器 x 64个传感器)
// 对应关系: 0-5: Accel(H/L xyz), 6-11: Gyro(H/L xyz)
struct IcmRawDataBuffer {
    uint8_t channels[12][64];
};

class Icm20602Manager {
public:
    static Icm20602Manager& getInstance() {
        static Icm20602Manager instance;
        return instance;
    }
    Icm20602Manager(const Icm20602Manager&) = delete;
    Icm20602Manager& operator=(const Icm20602Manager&) = delete;

    void init();
    void readAllSensors(IcmRawDataBuffer* pPacketBuffer);

private:
    Icm20602Manager() = default;
    void writeReg(uint8_t reg, uint8_t data);
    
    static const uint8_t ADDR_WRITE = 0xD0;
    static const uint8_t ADDR_READ  = 0xD1;
};

inline auto& ImuSensors = Icm20602Manager::getInstance();
