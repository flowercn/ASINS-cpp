#include "cpp_main.h"
#include "stm32f10x.h"
#include "main.h"
#include "ICM20602.h"
#include "Serial.h"
#include "GpioPin.h" // 引入刚才写的模板驱动

using LedRed   = PA<0>;
using LedGreen = PA<1>;

class ScopedIrqLock {
public:
    ScopedIrqLock() { __disable_irq(); }
    ~ScopedIrqLock() { __enable_irq(); }
    ScopedIrqLock(const ScopedIrqLock&) = delete;
    ScopedIrqLock& operator=(const ScopedIrqLock&) = delete;
};

class DoubleBufferManager {
public:
    DoubleBufferManager(uint8_t* b1, uint8_t* b2) 
        : buf1_(b1), buf2_(b2), current_write_ptr_(b1) {}
    uint8_t* try_acquire_write_buffer() {
        ScopedIrqLock lock; 
        if (current_write_ptr_ == s_dma_current_buffer || 
            current_write_ptr_ == s_dma_next_buffer) {
            return nullptr; 
        }
        return current_write_ptr_;
    }
    void swap() {
        current_write_ptr_ = (current_write_ptr_ == buf1_) ? buf2_ : buf1_;
    }

private:
    uint8_t* buf1_;
    uint8_t* buf2_;
    uint8_t* current_write_ptr_;
};

void cpp_entry(void) {
    DoubleBufferManager buffer_mgr(p_ping_buffer, p_pong_buffer);
    LedRed::reset();
    LedGreen::set();
    uint32_t loop_count = 0;
    while (1) {
        uint8_t* buffer = buffer_mgr.try_acquire_write_buffer();
        if (buffer) {
            auto* packet = reinterpret_cast<SerialImuPacket_t*>(buffer);
            ICM20602_ReadBurst_Bare(packet);
            Serial_SendBuffer(buffer);
            buffer_mgr.swap();
            loop_count++;
            if (loop_count >= 100) {
                LedRed::toggle();
                loop_count = 0;
            }
        }
    }
}
