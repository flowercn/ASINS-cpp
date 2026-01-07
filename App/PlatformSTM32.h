#pragma once
#include "ImuService.h"
#include "Serial.h"
#include "ICM20602.h"
#include <cstring> // for memcpy
#include "Timer.h"

// PlatformSTM32.h
#include "Serial.h"

class Stm32IoPort : public IIoPort {
private:
    uint8_t* _lockedBuf = nullptr; // 记录当前申请到的缓冲区

public:
    // 1. 直接申请 DMA 内存
    uint8_t* acquireTxBuffer() override {
        auto& serial = SerialManager::getInstance();
        
        // 先看 Active
        uint8_t* buf = serial.getActiveBuffer();
        if (!serial.isBufferBusy(buf)) {
            _lockedBuf = buf;
            return buf;
        }
        
        // 再看 Shadow
        buf = serial.getShadowBuffer();
        if (!serial.isBufferBusy(buf)) {
            _lockedBuf = buf;
            return buf;
        }
        
        // 都忙，没得用
        _lockedBuf = nullptr;
        return nullptr;
    }

    // 2. 填完数据后，触发 DMA
    bool commitTxBuffer(size_t len) override {
        if (!_lockedBuf) return false;

        auto& serial = SerialManager::getInstance();
        // 这里直接 transmit，数据已经在 acquireTxBuffer 返回的指针里了
        bool ret = serial.transmit(_lockedBuf, len); 
        
        _lockedBuf = nullptr; // 解锁
        return ret;
    }

    // 保留辅助函数
    bool isTxBusy() const override {
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
    static IcmRawDataBuffer rawData;

	outFrame.timestamp = GetHwTimestamp();
    Icm20602Manager::getInstance().readAllSensors(&rawData);

	for (int axis = 0; axis < 6; ++axis) {
        // 利用指针运算加速
        const uint8_t* ch_ptr = rawData.channels[axis * 2]; // High byte
        const uint8_t* cl_ptr = rawData.channels[axis * 2 + 1]; // Low byte
        
        for (int i = 0; i < SensorFrame::SENSORS; ++i) {
            // 直接拼接
            outFrame.data[axis][i] = static_cast<int16_t>((ch_ptr[i] << 8) | cl_ptr[i]);
        }
    }
}