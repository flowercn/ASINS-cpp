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


// 静态累加器：[6个轴][64个传感器]
// 0:Ax, 1:Ay, 2:Az, 3:Gx, 4:Gy, 5:Gz
static int32_t s_accumulator[6][64]; 
static int s_sample_count = 0;
static SerialImuPacket_t s_temp_buffer[12]; // 临时读取缓冲区

// 清空累加器
void Reset_Accumulators() {
    memset(s_accumulator, 0, sizeof(s_accumulator));
    s_sample_count = 0;
}

// 累加数据
static void Accumulate_Data(SerialImuPacket_t* raw_packets) {
    for (int sensor_idx = 0; sensor_idx < 64; sensor_idx++) {
        for (int axis = 0; axis < 6; axis++) {
            uint8_t high = raw_packets[axis * 2].ucData[sensor_idx];
            uint8_t low  = raw_packets[axis * 2 + 1].ucData[sensor_idx];
            int16_t value = (int16_t)((high << 8) | low);
            s_accumulator[axis][sensor_idx] += value;
        }
    }
    s_sample_count++;
}

// 计算平均值并写入 DMA 发送缓冲区
void Average_And_Write_To_Buffer(SerialImuPacket_t* tx_buffer) {
    if (s_sample_count == 0) return;
    // 初始化 Packet 的头和序号 
    ICM20602_PreInit_PacketBuffers((uint8_t*)tx_buffer, (uint8_t*)tx_buffer);
    for (int sensor_idx = 0; sensor_idx < 64; sensor_idx++) {
        for (int axis = 0; axis < 6; axis++) {
            int32_t avg_val = s_accumulator[axis][sensor_idx] / s_sample_count;
            tx_buffer[axis * 2].ucData[sensor_idx]     = (avg_val >> 8) & 0xFF;
            tx_buffer[axis * 2 + 1].ucData[sensor_idx] = avg_val & 0xFF;
        }
    }
    // 计算校验和
    for (uint8_t i = 0; i < 12; i++) {
        uint8_t checksum = 0;
        uint8_t* pBytes = (uint8_t*)&tx_buffer[i];
        for (uint8_t k = 0; k < SERIAL_PACKET_SIZE - 1; k++) {
            checksum += pBytes[k];
        }
        tx_buffer[i].ucChecksum = checksum;
    }
}

// ====================================================================
// 主入口
// ====================================================================
void cpp_entry(void) {
    DoubleBufferManager buffer_mgr(p_ping_buffer, p_pong_buffer);
    LedRed::reset();
    LedGreen::set();
    
    Reset_Accumulators();

    while (1) {
        // 获取 DMA 缓冲区
        uint8_t* tx_buffer = buffer_mgr.try_acquire_write_buffer();

        if (tx_buffer != nullptr) {
            // === DMA 空闲：准备发送 ===
            if (s_sample_count > 0) {
                // 有累加数据 -> 取平均并发走
                Average_And_Write_To_Buffer(reinterpret_cast<SerialImuPacket_t*>(tx_buffer));
            } else {
                // 刚启动第一次可能没数据 -> 读一次再发
                ICM20602_ReadBurst_Bare(reinterpret_cast<SerialImuPacket_t*>(tx_buffer));
            }

            Serial_SendBuffer(tx_buffer);
            buffer_mgr.swap();
            Reset_Accumulators();
            LedRed::toggle();
        } 
        else {
            // === DMA 忙碌：CPU 空闲 ===
            if (s_sample_count < 2) {
                ICM20602_ReadBurst_Bare(s_temp_buffer);
                Accumulate_Data(s_temp_buffer);
            } else {
            }
        }
    }
}
