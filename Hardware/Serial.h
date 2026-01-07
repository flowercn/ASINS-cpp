#pragma once
#include "stm32f10x.h"
#include <cstring>
#include <cstdio>
#include <cstdint>

#pragma pack(push, 1)
struct SerialImuPacket { 
    uint8_t head[2];     
    uint8_t serialNumber;
    uint8_t data[64];
    uint8_t checksum;
};
#pragma pack(pop)
static constexpr size_t SERIAL_PACKET_SIZE = sizeof(SerialImuPacket);

// ====================================================================
// 动态帧协议 (Unified Frame Protocol)
// ====================================================================
// 帧类型定义
enum class FrameType : uint8_t {
    Raw16  = 0x00,  // 全量帧 (int16 原码)
    Delta8 = 0x01   // 差分帧 (int8 差值)
};

#pragma pack(push, 1)
struct FrameHeader {
    uint8_t magic[2]; // 0xA5, 0x5A
    FrameType type;   // 0x00 或 0x01
    uint8_t frameCounter; // 对齐/计数器/状态位
	uint32_t timestamp;  //时间戳
};// 协议头 (4字节)

struct PayloadRaw {
    int16_t data[6][64]; 
};// Payload: 全量模式 (768 字节)

struct PayloadDelta {
    int8_t data[6][64];
};// Payload: 差分模式 (384 字节)
#pragma pack(pop)
static constexpr size_t MAX_FRAME_SIZE = sizeof(FrameHeader) + sizeof(PayloadRaw) + 1;

// ====================================================================
// SerialManager (支持变长 DMA)
// ====================================================================

class ScopedIrqLock {
public:
    ScopedIrqLock() { __disable_irq(); }
    ~ScopedIrqLock() { __enable_irq(); }
    ScopedIrqLock(const ScopedIrqLock&) = delete;
    ScopedIrqLock& operator=(const ScopedIrqLock&) = delete;
};

enum class SerialCommand : uint8_t {
    None = 0,
    Calibrate = 'C'
};

class SerialManager{
public:
	static SerialManager& getInstance(){
		static SerialManager instance;
		return instance;
	}
	SerialManager(const SerialManager&) = delete;
	SerialManager& operator=(const SerialManager&)=delete;
	
	void init(uint32_t baudrate);
	uint8_t* getActiveBuffer() { return s_buffer_active; }
    uint8_t* getShadowBuffer() { return s_buffer_shadow; }
	SerialCommand getCommand();
	bool transmit(uint8_t* pBuffer, size_t length);
	bool isBufferBusy(uint8_t* pBuf);
	void handleDmaIsr();
	void handleUartIsr();

private:
	SerialManager()=default;
	void configHardware(uint32_t bound);
	void configDma();

	volatile uint8_t* current_tx_ptr=nullptr;
	volatile uint8_t* next_tx_ptr=nullptr;
	size_t next_tx_length = 0;
	volatile uint8_t received_cmd=0;

	static uint8_t s_buffer_active[MAX_FRAME_SIZE];
    static uint8_t s_buffer_shadow[MAX_FRAME_SIZE];
};
inline auto& Serial = SerialManager::getInstance();
