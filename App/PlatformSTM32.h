#pragma once
#include "ImuService.h"
#include "Serial.h"
#include "ICM20602.h"
#include <cstring> // for memcpy

class Stm32IoPort : public IIoPort {
public:
    bool write(const uint8_t* data, size_t len) override {
        auto& serial = SerialManager::getInstance();       
        // 寻找空闲的 DMA 缓冲区 (Active 或 Shadow)
        uint8_t* targetBuf = serial.getActiveBuffer();
        
        if (serial.isBufferBusy(targetBuf)) {
            targetBuf = serial.getShadowBuffer();
            // 影子忙，说明发送太快了，丢弃或返回失败
            if (serial.isBufferBusy(targetBuf)) return false; 
        }
        std::memcpy(targetBuf, data, len); // 数据拷到DMA缓冲区     
        return serial.transmit(targetBuf, len); // 启动 DMA
    }

    bool isTxBusy() const override {
        // 判断两个缓冲区忙状态
        auto& serial = SerialManager::getInstance();
        return serial.isBufferBusy(serial.getActiveBuffer()) && 
               serial.isBufferBusy(serial.getShadowBuffer());
    }

    uint8_t readCommand() override {
        return static_cast<uint8_t>(SerialManager::getInstance().getCommand());
    }
};

// 硬件数据读取
static void FetchSensorData_STM32(SensorFrame& outFrame) {
    static SerialImuPacket rawPackets[12]; 
	static bool headers_inited = false;
    // 初始化 rawPackets 的头部信息
    if (!headers_inited) {
        Icm20602Manager::getInstance().initPacketHeaders(
            reinterpret_cast<uint8_t*>(rawPackets), 
            reinterpret_cast<uint8_t*>(rawPackets) // Ping/Pong 共用 buffer
        );
        headers_inited = true;
    }
    Icm20602Manager::getInstance().readAllSensors(rawPackets);

    // 格式转换：从 SerialImuPacket 转到 SensorFrame
    for (int axis = 0; axis < 3; ++axis) {
        for (int i = 0; i < SensorFrame::SENSORS; ++i) {
            uint8_t h = rawPackets[axis * 2].data[i];
            uint8_t l = rawPackets[axis * 2 + 1].data[i];
            outFrame.data[axis][i] = static_cast<int16_t>((h << 8) | l);
        }
    }
    for (int axis = 0; axis < 3; ++axis) {
        for (int i = 0; i < SensorFrame::SENSORS; ++i) {
            uint8_t h = rawPackets[6 + axis * 2].data[i];
            uint8_t l = rawPackets[6 + axis * 2 + 1].data[i];
            outFrame.data[3 + axis][i] = static_cast<int16_t>((h << 8) | l);
        }
    }
}