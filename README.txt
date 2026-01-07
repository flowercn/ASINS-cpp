这是一个为您量身定制的 Modern C++ 风格工程 README。

它基于您最新的代码架构（零拷贝、面向对象、Raw/Delta 自适应协议、无锁并行 I2C）编写，重点突出了**数据流转**和**文件职责**，确保任何接手的人（或未来的你自己）能一眼看懂。

---

# STM32 64通道高并发 IMU 遥测系统 (Modern C++ Architecture)

## 1. 项目概述 (Project Overview)

本项目是一个高性能嵌入式固件，运行于 **STM32F103 (144Pin)** 平台。旨在通过**并行 I2C (Parallel Bit-banging)** 技术，在一个时钟周期内同时采集 **64个 ICM-20602** 惯性传感器的数据。

系统采用 **Modern C++ (C++11/14)** 开发，核心特性包括：

* **极致性能**：利用 GPIO 端口宽总线特性，单次 I2C 读取耗时仅 ~0.3ms。
* **零拷贝架构 (Zero-Copy)**：从业务层生成数据到 DMA 发送，全程无内存拷贝。
* **自适应协议**：支持 **Raw16 (全量)** 和 **Delta8 (差分)** 两种模式，根据数据变化率自动切换，平衡带宽与精度。
* **高信噪比**：采用“全量采样-均值输出”策略，利用 DMA 发送间隙进行过采样平滑。

---

## 2. 软件架构与文件导读 (Architecture & Files)

工程采用严格的 **分层架构 (Layered Architecture)**，实现了业务逻辑与硬件驱动的彻底解耦。

### 2.1 顶层入口 (Application Layer)

* **`main.cpp`**
* **职责**：系统入口。负责硬件初始化 (`HwTimer`, `Serial`, `I2C`)，并注入依赖 (`ImuService` -> `Stm32IoPort`)。
* **主循环**：死循环执行 `FetchSensorData_STM32` (获取原始数据) -> `g_imuService.process` (业务处理)。



### 2.2 业务逻辑层 (Service Layer) - *核心大脑*

此层纯 C++ 实现，**不依赖具体硬件**，只处理数据逻辑。

* **`ImuService.h / .cpp`**
* **职责**：管理系统状态（正常/校准）、数据累加、平滑滤波、差分计算、协议打包。
* **关键逻辑**：
* `accumulateData()`: 无论串口是否忙，持续累加传感器数据（过采样）。
* `sendDataPacket()`: 向底层申请 buffer，决定发送 Raw 还是 Delta 帧，并提交发送。


* **依赖**：通过 `IIoPort` 接口与底层通信，实现了**零拷贝**的核心逻辑。



### 2.3 硬件适配层 (Adapter Layer) - *桥梁*

* **`PlatformSTM32.h`**
* **职责**：实现 `IIoPort` 接口。将业务层的抽象请求（如 `acquireTxBuffer`）映射到具体的 STM32 驱动调用。
* **数据转换**：其中的 `FetchSensorData_STM32` 函数负责将 `MyI2C` 读取的 `Channel` 格式（物理排列）转换为 `SensorFrame` 格式（逻辑排列）。



### 2.4 驱动层 (Driver Layer) - *脏活累活*

* **`MyI2C.h / .cpp`** (并行 I2C 核心)
* 利用 C++ 模板元编程 (`GpioPin.h`) 操作 GPIO BSRR/IDR 寄存器。
* 实现 64 路 SDA 和 2 组 SCL 的并行时序控制。


* **`ICM20602.h / .cpp`**
* 传感器寄存器定义与配置流程（初始化、设置量程等）。


* **`Serial.h / .cpp`** (DMA 串口)
* 维护 **双缓冲区 (Active/Shadow)**。
* 提供 `acquire` (获取指针) 和 `transmit` (启动 DMA) 接口。


* **`Timer.h / .cpp`**
* 维护 0.1ms 级精度的全局硬件时间戳。



---

## 3. 核心数据流详解 (Data Flow Pipeline)

整个系统的数据流转是**同步采样、异步发送**的流水线：

### 阶段 1：并行采集 (Sampling)

1. **触发**：`main` 循环调用 `FetchSensorData_STM32`。
2. **动作**：`MyI2C` 驱动拉动 SCL，**同时**读取 64 个 GPIO 引脚的电平。
3. **存储**：数据被存入 `IcmRawDataBuffer` (12个寄存器 × 64路)。
4. **映射**：适配层将其重组为 `SensorFrame` (6轴 × 64路) 并打上时间戳。

### 阶段 2：逻辑处理 (Processing)

1. **累加**：`ImuService::accumulateData` 接收一帧数据，加入累加器 `_accumulator`。
2. **判断**：
* 若串口**忙**：直接返回，继续下一轮采样（数据保留在累加器中，实现平滑滤波）。
* 若串口**空闲**：进入发送流程。



### 阶段 3：零拷贝发送 (Zero-Copy Transmission)

1. **申请内存**：`ImuService` 调用 `_io.acquireTxBuffer()`。
* `PlatformSTM32` 从 `SerialManager` 请求当前的 DMA 缓冲区指针（Active 或 Shadow）。


2. **打包填入**：
* **Raw模式**：将 `int16` 均值直接写入申请到的 DMA 内存。
* **Delta模式**：计算 `当前均值 - 上次发送值`，将 `int8` 差值写入 DMA 内存。


3. **提交发送**：`ImuService` 调用 `_io.commitTxBuffer()`。
* `SerialManager` 启动 DMA 传输。CPU 立即释放，回到阶段 1 继续采样。



---

## 4. 通信协议 (Unified Frame Protocol)

协议设计为变长帧，由 **8字节定长包头** + **变长 Payload** 组成。

### 4.1 帧头定义 (8 Bytes)

所有帧类型共享相同的头部结构：

```cpp
struct FrameHeader {
    uint8_t  magic[2];     // 固定为 0xA5, 0x5A
    uint8_t  type;         // 帧类型：0x00 (Raw16) 或 0x01 (Delta8)
    uint8_t  frameCounter; // 循环计数器 (0-255)
    uint32_t timestamp;    // 硬件时间戳 (单位 0.1ms)
};

```

### 4.2 帧类型 A：全量帧 (Raw16)

* **Type**: `0x00`
* **Payload**: `768 Bytes` (6轴 × 64传感器 × 2字节 `int16_t`)
* **Total Size**: 777 Bytes
* **用途**: 关键帧、数据剧烈变化时、或初始化阶段。发送传感器的原始物理量（均值）。

### 4.3 帧类型 B：差分帧 (Delta8)

* **Type**: `0x01`
* **Payload**: `384 Bytes` (6轴 × 64传感器 × 1字节 `int8_t`)
* **Total Size**: 393 Bytes
* **用途**: 数据平稳变化时。发送 `(当前值 - 上次发送值)`。极大节省带宽。

### 4.4 校验

帧尾附加 **1 Byte Checksum** (Header + Payload 的累加和)。

---

## 5. 性能指标 (Performance)

* **波特率**: 921600 bps (支持超频至 2Mbps+)
* **采样率**: > 3000 Hz (I2C 物理采样)
* **输出帧率**:
* 静态 (Delta模式): **~230 Hz**
* 动态 (Raw模式): **~130 Hz**


* **延迟**: < 5ms

## 6. 编译与注意事项

* **编译器**: 需要支持 **C++11** 或更高标准。
* **优化等级**: 建议 `-O2` 或 `-O3`，因为 `GpioPin.h` 极其依赖内联优化。
* **硬件依赖**: 必须使用 STM32F103 **Z系列 (144脚)** 芯片，因为 `MyI2C` 强依赖 GPIOF/GPIOG 端口。**严禁在 C8T6/RCT6 上运行，否则会 HardFault。**
