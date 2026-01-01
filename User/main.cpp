#include "stm32f10x.h"
#include "Serial.h"
#include "ICM20602.h"
#include "GpioPin.h" 
#include <cstring>

using LedRed   = PA<0>;
using LedGreen = PA<1>;

class ImuDataProcessor {
public:
    ImuDataProcessor() {
        resetAccumulators();
    }
	
    void startCalibration() {
        currentState_ = State::Calibrating;
        calibFrameCount_ = 0;
        totalRealSamples_ = 0;
        std::memset(offsets_, 0, sizeof(offsets_));
        resetAccumulators();
        
        LedGreen::reset(); 
        LedRed::set();
    }
	
    bool update(uint8_t* txBuffer) {
        // 采样
        if (sampleCount_ < 1) {
            IcmSensors.readAllSensors(tempBuffer_);//使用临时缓冲区读取
            accumulate(tempBuffer_);
        }
        // 状态分发逻辑
        if (currentState_ == State::Calibrating) {
            handleCalibration();
            return false; // 标定期间不发送数据
        } else {
            return handleNormalOperation(txBuffer);
        }
    }

private:
    enum class State {
        Normal,
        Calibrating
    };
    // === 常量定义 ===
    static constexpr int CALIB_TARGET_FRAMES = 200;
    static constexpr size_t SENSOR_COUNT = 64;
    static constexpr size_t AXIS_COUNT = 6;

    // === 成员变量 ===
    int32_t accumulator_[AXIS_COUNT][SENSOR_COUNT]; 
    int32_t offsets_[AXIS_COUNT][SENSOR_COUNT] = {0}; 
    SerialImuPacket tempBuffer_[12]; // 临时读取缓冲区
    
    int sampleCount_ = 0;
    State currentState_ = State::Normal;
    bool isCalibrated_ = false;
    
    int calibFrameCount_ = 0;     
    int32_t totalRealSamples_ = 0;

    // === 内部辅助函数 ===
    void resetAccumulators() {
        std::memset(accumulator_, 0, sizeof(accumulator_));
        sampleCount_ = 0;
    }

    void accumulate(const SerialImuPacket* rawPackets) {
        for (size_t sensorIdx = 0; sensorIdx < SENSOR_COUNT; sensorIdx++) {
            for (size_t axis = 0; axis < AXIS_COUNT; axis++) {
                uint8_t high = rawPackets[axis * 2].data[sensorIdx];
                uint8_t low  = rawPackets[axis * 2 + 1].data[sensorIdx];
                int16_t value = static_cast<int16_t>((high << 8) | low);
                accumulator_[axis][sensorIdx] += value;
            }
        }
        sampleCount_++;
    }

    void handleCalibration() {
        if (sampleCount_ > 0) {
            // 仅累加陀螺仪 (Axis 3, 4, 5)
            for (size_t i = 0; i < SENSOR_COUNT; i++) {
                for (size_t j = 3; j < AXIS_COUNT; j++) {
                    offsets_[j][i] += accumulator_[j][i]; 
                }
            }
            totalRealSamples_ += sampleCount_;
            calibFrameCount_++;
            resetAccumulators(); 
        }

        if (calibFrameCount_ >= CALIB_TARGET_FRAMES) {
            finishCalibration();
        }
    }

    void finishCalibration() {
        if (totalRealSamples_ > 0) {
            for (size_t i = 0; i < SENSOR_COUNT; i++) {
                for (size_t j = 3; j < AXIS_COUNT; j++) {
                    offsets_[j][i] /= totalRealSamples_; 
                }
            }
        }
        isCalibrated_ = true;
        currentState_ = State::Normal;
        LedRed::reset();
        LedGreen::set();
    }

    bool handleNormalOperation(uint8_t* txBuffer) {
        // 只有当：(1)有数据 AND (2)串口不忙 时，才进行处理
        if (sampleCount_ > 0) {
            if (!Serial.isBufferBusy(txBuffer)) {
                fillTransmitBuffer(reinterpret_cast<SerialImuPacket*>(txBuffer));
                Serial.transmit(txBuffer);
                resetAccumulators();
                return true; // 通知外部：发送成功，请切换 Buffer
            }
        }
        return false;
    }

    void fillTransmitBuffer(SerialImuPacket* outPackets) {
        // 调用 ICM 类的功能初始化包头
        IcmSensors.initPacketHeaders((uint8_t*)outPackets, (uint8_t*)outPackets);
        // 计算平均值并扣除零偏
        for (size_t sensorIdx = 0; sensorIdx < SENSOR_COUNT; sensorIdx++) {
            for (size_t axis = 0; axis < AXIS_COUNT; axis++) {
                int32_t avgVal = accumulator_[axis][sensorIdx] / sampleCount_;
                // 陀螺仪减去零偏估计值
                if (isCalibrated_ && axis >= 3) {
                    avgVal -= offsets_[axis][sensorIdx];
                }
                // 填入缓冲区
                outPackets[axis * 2].data[sensorIdx]     = (avgVal >> 8) & 0xFF;
                outPackets[axis * 2 + 1].data[sensorIdx] = avgVal & 0xFF;
            }
        }
        // 计算校验和
        for (size_t i = 0; i < 12; i++) {
            uint8_t checksum = 0;
            uint8_t* pBytes = (uint8_t*)&outPackets[i];
            for (size_t k = 0; k < SERIAL_PACKET_SIZE - 1; k++) {
                checksum += pBytes[k];
            }
            outPackets[i].checksum = checksum;
        }
    }
};

ImuDataProcessor g_imuProcessor;

int main(void)
{
    // 硬件初始化
    LedRed::initOutput();
    LedGreen::initOutput();
    LedRed::reset();
    LedGreen::set();

    Serial.init(921600);
    IcmSensors.init(); 

	uint8_t* activeBuffer = Serial.getActiveBuffer();
    uint8_t* shadowBuffer = Serial.getShadowBuffer();
	uint8_t* currentWriteBuffer = activeBuffer;

    // 主循环
    while (1) {
        if (Serial.getCommand() == SerialCommand::Calibrate) { 
            g_imuProcessor.startCalibration();     
        }
		if (g_imuProcessor.update(currentWriteBuffer)) {
            currentWriteBuffer = (currentWriteBuffer == activeBuffer) ? shadowBuffer : activeBuffer;
        }
    }
}
