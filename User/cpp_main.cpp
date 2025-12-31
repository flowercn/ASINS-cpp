#include "cpp_main.h"
#include "stm32f10x.h"
#include "main.h"
#include "ICM20602.h"
#include "Serial.h"
#include "GpioPin.h" 

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

// ====================================================================
// 全局变量定义
// ====================================================================

// 静态累加器：[6个轴][64个传感器]
static int32_t s_accumulator[6][64]; 
static int s_sample_count = 0;
static SerialImuPacket_t s_temp_buffer[12]; // 临时读取缓冲区

// 零偏相关
static int32_t s_offsets[6][64] = {0}; 
static bool s_is_calibrated = false;   

// 标定状态机
enum State {
    STATE_NORMAL,
    STATE_CALIBRATING
};
static State s_current_state = STATE_NORMAL;
static int s_calib_frame_count = 0;     
static int32_t s_total_real_samples = 0; 
static const int CALIB_TARGET_FRAMES = 200; 

// ====================================================================
// 核心函数
// ====================================================================

void Reset_Accumulators() {
    memset(s_accumulator, 0, sizeof(s_accumulator));
    s_sample_count = 0;
}

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

// 计算平均值 -> 减去零偏 -> 写入 buffer -> 计算校验和
void Average_And_Write_To_Buffer(SerialImuPacket_t* tx_buffer) {
    if (s_sample_count == 0) return;

    // 1. 初始化 Packet 的头和序号 
    ICM20602_PreInit_PacketBuffers((uint8_t*)tx_buffer, (uint8_t*)tx_buffer);

    // 2. 计算平均值并扣除零偏
    for (int sensor_idx = 0; sensor_idx < 64; sensor_idx++) {
        for (int axis = 0; axis < 6; axis++) {
            // A. 算出当前帧的原始平均值
            int32_t avg_val = s_accumulator[axis][sensor_idx] / s_sample_count;
            
            // B. 减去零偏
            if (s_is_calibrated) {
                avg_val -= s_offsets[axis][sensor_idx];
            }

            // C. 填入发送缓冲区
            tx_buffer[axis * 2].ucData[sensor_idx]     = (avg_val >> 8) & 0xFF;
            tx_buffer[axis * 2 + 1].ucData[sensor_idx] = avg_val & 0xFF;
        }
    }

    // 3. 计算校验和
    for (uint8_t i = 0; i < 12; i++) {
        uint8_t checksum = 0;
        uint8_t* pBytes = (uint8_t*)&tx_buffer[i];
        for (uint8_t k = 0; k < SERIAL_PACKET_SIZE - 1; k++) {
            checksum += pBytes[k];
        }
        tx_buffer[i].ucChecksum = checksum;
    }
}

// 针对单次读取的 Apply_Offsets (修复了校验和缺失的问题)
static void Apply_Offsets_For_Raw(SerialImuPacket_t* tx_buffer) {
    // 1. 减零偏
    if (s_is_calibrated) {
        for (int sensor_idx = 0; sensor_idx < 64; sensor_idx++) {
            for (int axis = 0; axis < 6; axis++) {
                uint8_t h = tx_buffer[axis*2].ucData[sensor_idx];
                uint8_t l = tx_buffer[axis*2+1].ucData[sensor_idx];
                int16_t raw = (int16_t)((h << 8) | l);
                
                int32_t corrected = raw - s_offsets[axis][sensor_idx];
                
                tx_buffer[axis*2].ucData[sensor_idx]     = (corrected >> 8) & 0xFF;
                tx_buffer[axis*2+1].ucData[sensor_idx]   = corrected & 0xFF;
            }
        }
    }
    
    // 因为 ReadBurst 里删了校验，如果不算，单次包会被上位机丢弃
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
        // 1. 处理指令
        uint8_t cmd = Serial_GetCommand();
        if (cmd == 'C') { 
            s_current_state = STATE_CALIBRATING;
            s_calib_frame_count = 0;
            s_total_real_samples = 0; 
            memset(s_offsets, 0, sizeof(s_offsets));
            Reset_Accumulators(); 
            LedGreen::reset(); 
            LedRed::set();      
        }
        
        uint8_t* tx_buffer = buffer_mgr.try_acquire_write_buffer();
        
        if (tx_buffer != nullptr) {
            // === DMA 空闲 ===
            if (s_current_state == STATE_CALIBRATING) {
                // --- 标定模式 ---
                
                // 防止死锁保底：如果太快没读到数据，强制读一次
                if (s_sample_count == 0) {
                    ICM20602_ReadBurst_Bare(s_temp_buffer);
                    Accumulate_Data(s_temp_buffer);
                }

                if (s_sample_count > 0) {
                    for(int i=0; i<64; i++) 
                        for(int j=3; j<6; j++) 
                            s_offsets[j][i] += s_accumulator[j][i]; 
                    
                    s_total_real_samples += s_sample_count;
                    s_calib_frame_count++;
                }

                if (s_calib_frame_count >= CALIB_TARGET_FRAMES) {
                    // === 标定结束：计算最终零偏 ===
                    if (s_total_real_samples > 0) {
                        for(int i=0; i<64; i++) {
                            // 只有陀螺仪 (Axis 3,4,5) 需要扣除静止零偏
                            for(int j=3; j<6; j++) {
                                s_offsets[j][i] /= s_total_real_samples; 
                            }
                        }
                    }
                    
                    s_is_calibrated = true;
                    s_current_state = STATE_NORMAL;
                    LedRed::reset();
                    LedGreen::set();
                }
                buffer_mgr.swap(); 
            }
            else {
                // --- 正常模式 ---
                if (s_sample_count > 0) {
                    Average_And_Write_To_Buffer(reinterpret_cast<SerialImuPacket_t*>(tx_buffer));
                } else {
                    ICM20602_ReadBurst_Bare(reinterpret_cast<SerialImuPacket_t*>(tx_buffer));
                    // 补上单次采样的校验和
                    Apply_Offsets_For_Raw(reinterpret_cast<SerialImuPacket_t*>(tx_buffer));
                }
                Serial_SendBuffer(tx_buffer);
                buffer_mgr.swap();
            }
            Reset_Accumulators();
        } 
        else {
            // === CPU 空闲 ===
            if (s_sample_count < 2) {
                ICM20602_ReadBurst_Bare(s_temp_buffer);
                Accumulate_Data(s_temp_buffer);
            } 
        }
    }
}