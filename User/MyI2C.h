#pragma once
#include "stm32f10x.h"
#include <cstddef>

static constexpr size_t I2C_NUM = 64; 

class ParallelI2CManager {
public:
    static ParallelI2CManager& getInstance() {
        static ParallelI2CManager instance;
        return instance;
    }
    
    ParallelI2CManager(const ParallelI2CManager&) = delete;
    ParallelI2CManager& operator=(const ParallelI2CManager&) = delete;

    void init(); 
    void start();
    void stop();
    
    void sendByte(uint8_t byte, uint8_t* pAckBuffer);
    void receiveByte(uint8_t* pRxBuffer, bool sendAck);

    bool hasError() const { return error_detected_; }
    void clearError() { error_detected_ = false; }

private:
    ParallelI2CManager() = default;
    volatile bool error_detected_ = false;
};
inline auto& ParallelI2C = ParallelI2CManager::getInstance();
