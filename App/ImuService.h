#pragma once
#include <stdint.h>  
#include <stddef.h>
#include "Serial.h"

template<typename T, size_t N>
struct LiteArray {
    T elems[N];
    // 支持下标访问 []
    T& operator[](size_t i) { return elems[i]; }
    const T& operator[](size_t i) const { return elems[i]; }
    // 支持获取大小
    static constexpr size_t size() { return N; }
    // 支持 .data() 获取裸指针
    T* data() { return elems; }
    const T* data() const { return elems; }
    // 支持范围 for 循环
    T* begin() { return elems; }
    T* end() { return elems + N; }
    const T* begin() const { return elems; }
    const T* end() const { return elems + N; }
};

// 传感器数据帧
struct SensorFrame {
    static constexpr int SENSORS = 64;
    static constexpr int AXIS = 6;
    LiteArray<LiteArray<int16_t, SENSORS>, AXIS> data;
    void clear() {
        // 利用结构体赋值特性清零
        *this = SensorFrame{}; 
    }
};

// IO 端口
class IIoPort {
public:
    virtual ~IIoPort() = default; // 发送数据 (非阻塞)    
    virtual bool write(const uint8_t* data, size_t len) = 0; // 发送忙检测
    virtual bool isTxBusy() const = 0;
    virtual uint8_t readCommand() = 0; // 读取指令 (返回 0 表示无指令)
};

// Service
class ImuService {
public:    
    ImuService(IIoPort& ioPort);// 构造函数：必须注入 IO 端口  
    void process(const SensorFrame& frame); // 处理一帧数据
private:
    IIoPort& _io; // 接口引用

	// --- 状态定义 ---
    enum class State { Normal, Calibrating };
    State _state = State::Normal;
	
    // --- 内部常量 ---
    static constexpr int SENSORS = SensorFrame::SENSORS;
    static constexpr int AXIS = SensorFrame::AXIS;
    static constexpr int KEYFRAME_INTERVAL = 100;    

    // --- 算法缓存区 ---
    int32_t _accumulator[AXIS][SENSORS];   // 累加器
    int32_t _offsets[AXIS][SENSORS];       // 校准偏移
    int32_t _currentVals[AXIS][SENSORS];   // 当前均值
    int32_t _lastSentVals[AXIS][SENSORS];  // 差分基准值

    int _sampleCount = 0;
    int _calibCount = 0;
    int _framesSinceKeyframe = KEYFRAME_INTERVAL; // 初始强制发关键帧
    bool _isCalibrated = false;

    // --- 内部逻辑方法 ---
    void accumulateData(const SensorFrame& frame);
    
    void runNormalLogic();
    void runCalibrationLogic();
    
    void enterCalibration();
    void finishCalibration();

    void calculateAverages();
    void sendDataPacket();
    
    // --- 协议打包 ---
    bool shouldSendRaw();
    size_t fillRawPacket(uint8_t* buf);
    size_t fillDeltaPacket(uint8_t* buf);

    // --- 工具 ---
    void resetAcc();
    void resetOffsets();
};