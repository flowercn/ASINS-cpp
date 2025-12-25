#pragma once

#include "stm32f10x.h"
#include "ICM20602_Reg.h"
#include "MyI2C.h"
#include "Serial.h"

#define ICM20602_REG_COUNT  12
// ICM20602 I2C 写地址
#define ICM20602_I2C_ADDRESS  0xD0
extern "C" {
void ICM20602_PreInit_PacketBuffers(uint8_t* ping_buf, uint8_t* pong_buf);
int ICM20602_Init_Bare();
int ICM20602_ReadBurst_Bare(SerialImuPacket_t* pPacketBuffer);
} // extern "C"
