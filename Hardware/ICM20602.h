#pragma once

#include "stm32f10x.h"
#include "ICM20602_Reg.h"
#include "MyI2C.h"
#include "Serial.h"

static constexpr uint8_t ICM20602_I2C_ADDRESS = 0xD0;

class Icm20602Manager {
public:
    static Icm20602Manager& getInstance() {
        static Icm20602Manager instance;
        return instance;
    }
    Icm20602Manager(const Icm20602Manager&) = delete;
    Icm20602Manager& operator=(const Icm20602Manager&) = delete;

    void init();
    void initPacketHeaders(uint8_t* ping_buf, uint8_t* pong_buf);
    void readAllSensors(SerialImuPacket* pPacketBuffer);

private:
    Icm20602Manager() = default;
    void writeReg(uint8_t reg, uint8_t data);
    
    static const uint8_t ADDR_WRITE = 0xD0;
    static const uint8_t ADDR_READ  = 0xD1;
};

inline auto& IcmSensors = Icm20602Manager::getInstance();

extern "C" {
	void ICM20602_Init_Bare();
}