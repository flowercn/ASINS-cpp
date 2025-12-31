/**
 * @file Serial.h
 * @brief 串口通信及数据包结构定义模块
 * @details 定义了与上位机通信的数据包结构 SerialImuPacket_t，
 *          并声明了串口初始化和DMA发送相关的函数。
 * @author DYH
 * @date 2025-08-30/2025-12-31
 */

#pragma once

#include "stm32f10x.h"
#include <cstring>
#include <cstdio>

#pragma pack(push, 1)
typedef struct {
    uint8_t ucHead[2];        //!< 包头同步字 (0xA5, 0x5A)
    uint8_t ucSerialNumber;   //!< 包序号 (0x01 - 0x08)
    uint8_t ucData[64];       //!< 主要数据负载
    uint8_t ucChecksum;       //!< 校验位
} SerialImuPacket_t;
#pragma pack(pop)

#define SERIAL_PACKET_SIZE  (sizeof(SerialImuPacket_t))
#define IMU_REG_COUNT       (12)
#define DMA_BUFFER_SIZE     (IMU_REG_COUNT * SERIAL_PACKET_SIZE)

class SerialManager{
public:
	static SerialManager& getInstance(){
		static SerialManager instance;
		return instance;
	}
	SerialManager(const SerialManager&) = delete;
	SerialManager& operator=(const SerialManager&)=delete;
	
	void init(uint32_t baudrate);
	bool transmit(uint8_t* pBuffer);
	uint8_t getCommand();
	uint8_t* getActiveBuffer() { return s_buffer_active; }
    uint8_t* getShadowBuffer() { return s_buffer_shadow; }
	
	bool isBufferBusy(uint8_t* pBuf) {
        return (current_tx_ptr == pBuf) || (next_tx_ptr == pBuf);
    }
	
	void handleDmaIsr();
	void handleUartIsr();
private:
	SerialManager()=default;
	void configHardware(uint32_t bound);
	void configDma();

	volatile uint8_t* current_tx_ptr=nullptr;
	volatile uint8_t* next_tx_ptr=nullptr;
	volatile uint8_t received_cmd=0;

	static uint8_t s_buffer_active[DMA_BUFFER_SIZE];
    static uint8_t s_buffer_shadow[DMA_BUFFER_SIZE];
};
inline auto& Serial = SerialManager::getInstance();
