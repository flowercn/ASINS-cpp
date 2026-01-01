/**
 * @file Serial.h
 * @brief 串口通信及数据包结构定义模块
 * @details 定义了与上位机通信的数据包结构 SerialImuPacket，
 *          并声明了串口初始化和DMA发送相关的函数。
 * @author DYH
 * @date 2025-08-30/2025-12-31
 */

#pragma once

#include "stm32f10x.h"
#include <cstring>
#include <cstdio>

class ScopedIrqLock {
public:
    ScopedIrqLock() { __disable_irq(); }
    ~ScopedIrqLock() { __enable_irq(); }
    ScopedIrqLock(const ScopedIrqLock&) = delete;
    ScopedIrqLock& operator=(const ScopedIrqLock&) = delete;
};

#pragma pack(push, 1)
struct SerialImuPacket { 
    uint8_t head[2];     
    uint8_t serialNumber;
    uint8_t data[64];
    uint8_t checksum;
};
#pragma pack(pop)

enum class SerialCommand : uint8_t {
    None = 0,
    Calibrate = 'C'
};

static constexpr size_t SERIAL_PACKET_SIZE  (sizeof(SerialImuPacket));
static constexpr size_t IMU_REG_COUNT = 12;
static constexpr size_t DMA_BUFFER_SIZE = IMU_REG_COUNT * SERIAL_PACKET_SIZE;

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
	SerialCommand getCommand();
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
