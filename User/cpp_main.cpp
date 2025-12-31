#include "cpp_main.h"
#include "stm32f10x.h"
#include "main.h"
#include "ICM20602.h"
#include "Serial.h"
#include "GpioPin.h" 

using LedRed   = PA<0>;
using LedGreen = PA<1>;

// ====================================================================
// 全局变量定义
// ====================================================================

// 静态累加器：[6个轴][64个传感器]
static int32_t s_accumulator[6][64]; 
static int s_sample_count = 0;
static SerialImuPacket_t s_temp_buffer[12]; // 临时读取缓冲区

// 零偏相关 (虽然开了 [6][64]，但只会用到后3行 [3-5][64] 存陀螺仪)
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
            
            // B. 减去零偏 (仅针对陀螺仪: axis 3,4,5)
            // 源头修正：严格限制只修陀螺仪，防止误伤加速度计
            if (s_is_calibrated && axis >= 3) {
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

// 针对单次读取的 Apply_Offsets
static void Apply_Offsets_For_Raw(SerialImuPacket_t* tx_buffer) {
    if (s_is_calibrated) {
        for (int sensor_idx = 0; sensor_idx < 64; sensor_idx++) {
            // 源头修正：循环可以直接从 axis=3 开始，跳过加计
            for (int axis = 3; axis < 6; axis++) {
                uint8_t h = tx_buffer[axis*2].ucData[sensor_idx];
                uint8_t l = tx_buffer[axis*2+1].ucData[sensor_idx];
                int16_t raw = (int16_t)((h << 8) | l);
                
                int32_t corrected = raw - s_offsets[axis][sensor_idx];
                
                tx_buffer[axis*2].ucData[sensor_idx]     = (corrected >> 8) & 0xFF;
                tx_buffer[axis*2+1].ucData[sensor_idx]   = corrected & 0xFF;
            }
        }
    }
    
    // 补全校验和
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
    // 1. 获取 Serial 已经分配好的两个缓冲区地址
    uint8_t* p_pingpong_bufs[2];
    p_pingpong_bufs[0] = Serial.getActiveBuffer();
    p_pingpong_bufs[1] = Serial.getShadowBuffer();
    uint8_t  buf_idx = 0; 

    LedRed::reset();
    LedGreen::set();
    Reset_Accumulators();

    while (1) {
        // 1. 处理指令
        uint8_t cmd = Serial.getCommand();
        if (cmd == 'C') { 
            s_current_state = STATE_CALIBRATING;
            s_calib_frame_count = 0;
            s_total_real_samples = 0; 
            memset(s_offsets, 0, sizeof(s_offsets));
            Reset_Accumulators(); 
            LedGreen::reset(); 
            LedRed::set();      
        }
        
        uint8_t* current_tx_ptr = p_pingpong_bufs[buf_idx];

        if (s_current_state == STATE_CALIBRATING) {
            // === 标定模式 (逻辑保持不变) ===
            if (s_sample_count == 0) {
                ICM20602_ReadBurst_Bare(s_temp_buffer);
                Accumulate_Data(s_temp_buffer);
            }

            if (s_sample_count > 0) {
                // 仅累加陀螺仪
                for(int i=0; i<64; i++) {
                    for(int j=3; j<6; j++) {
                        s_offsets[j][i] += s_accumulator[j][i]; 
                    }
                }
                s_total_real_samples += s_sample_count;
                s_calib_frame_count++;
                Reset_Accumulators(); 
            }

            if (s_calib_frame_count >= CALIB_TARGET_FRAMES) {
                // 结算标定
                if (s_total_real_samples > 0) {
                    for(int i=0; i<64; i++) {
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
        }
        else {
            // === 正常模式 ===
            
            // 1. 始终尝试采样 (防止数据断流)
            if (s_sample_count < 1) {
                ICM20602_ReadBurst_Bare(s_temp_buffer);
                Accumulate_Data(s_temp_buffer);
            } 
            
            // 2. 只有当：(1)有数据 AND (2)串口不忙 时，才进行发送操作
            if (s_sample_count > 0) {
                // 【关键逻辑】检查当前要写的缓冲区是否正在被 DMA 占用
                if (!Serial.isBufferBusy(current_tx_ptr)) {
                    
                    // A. 不忙 -> 放心写入数据
                    Average_And_Write_To_Buffer(reinterpret_cast<SerialImuPacket_t*>(current_tx_ptr));
                    
                    // B. 启动 DMA
                    Serial.transmit(current_tx_ptr);
                    
                    // C. 切换到下一块缓冲区
                    buf_idx = !buf_idx;
                    Reset_Accumulators();
                }
                // D. 如果忙 -> 什么都不做，保留当前 Accumulator 数据，下一轮循环再试
            }
        }
    }
}
