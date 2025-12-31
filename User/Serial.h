/**
 * @file Serial.h
 * @brief 串口通信及数据包结构定义模块
 * @details 定义了与上位机通信的数据包结构 SerialImuPacket_t，
 *          并声明了串口初始化和DMA发送相关的函数。
 * @author dengyihang
 * @date 2025-08-30
 */

#pragma once

#include "stm32f10x.h"
#include <cstring>
#include <cstdio>

// ========================================
// 核心数据结构与宏定义
// ========================================

/**
 * @brief IMU数据包结构体
 * @details 定义了通过串口发送的单个数据包的格式。
 *          使用 #pragma pack(1) 强制 1 字节对齐，避免编译器自动添加填充字节。
 *          结构体大小：2 + 1 + 64 + 1 = 68 字节（显式添加保留字节对齐到 4 字节边界）
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t ucHead[2];        //!< 包头同步字 (0xA5, 0x5A)
    uint8_t ucSerialNumber;   //!< 包序号 (0x01 - 0x08)
    uint8_t ucData[64];       //!< 主要数据负载
    uint8_t ucChecksum;       //!< 保留字节（用于 4 字节对齐，提高效率）
} SerialImuPacket_t;
#pragma pack(pop)

// 单个IMU数据包的大小（字节）
#define SERIAL_PACKET_SIZE  (sizeof(SerialImuPacket_t))

// 一次完整传输包含的数据包数量（升级为完整 6 轴 x 2 字节 = 12 包）
#define IMU_REG_COUNT       (12)

// DMA单次传输的总大小（一个乒乓缓冲区的大小）
#define DMA_BUFFER_SIZE     (IMU_REG_COUNT * SERIAL_PACKET_SIZE)

// ========================================
// 外部可用变量
// ========================================

// Ping-Pong 缓冲区指针
extern uint8_t* p_ping_buffer;
extern uint8_t* p_pong_buffer;

// DMA 发送缓冲区状态
extern volatile uint8_t* s_dma_current_buffer;
extern volatile uint8_t* s_dma_next_buffer;

// ========================================
// 公共 API 函数声明
// ========================================

extern "C" {

uint8_t Serial_GetCommand(void);
/**
 * @brief 初始化串口硬件
 * @param ulBaudrate 波特率 (例如 921600)
 */
void Serial_Init(uint32_t ulBaudrate);

/**
 * @brief 发送一个数据缓冲区（非阻塞）
 * @details 使用DMA的乒乓缓冲机制发送一个完整的DMA缓冲区。
 *          函数将缓冲区指针加入发送队列后立即返回。
 * @param pBuffer 指向要发送的缓冲区的指针（大小应为 DMA_BUFFER_SIZE）
 * @return 0表示成功加入发送队列, -1表示缓冲区已满
 */
int Serial_SendBuffer(uint8_t* pBuffer);

} // extern "C"
