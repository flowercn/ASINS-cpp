#include "ICM20602.h"
#include <string.h>

// I2C应答信号
static uint8_t g_ucAck[I2C_NUM];

// =================================================================================
// 升级点 1: 扩展序列号映射 (12个包 = 6轴 x 2字节)
// =================================================================================
static const uint8_t g_ucSerialMap[] = {
    0x01, 0x02, // Accel X High, Low
    0x03, 0x04, // Accel Y High, Low
    0x05, 0x06, // Accel Z High, Low
    0x07, 0x08, // Gyro X High, Low
    0x09, 0x0A, // Gyro Y High, Low
    0x0B, 0x0C  // Gyro Z High, Low
};

// =================================================================================
// 升级点 2: 完整的寄存器映射表 (读取 3轴加速度 + 3轴陀螺仪)
// =================================================================================
static const uint8_t g_ucRegisterMap[] = {
    ICM20602_ACCEL_XOUT_H, ICM20602_ACCEL_XOUT_L,
    ICM20602_ACCEL_YOUT_H, ICM20602_ACCEL_YOUT_L,
    ICM20602_ACCEL_ZOUT_H, ICM20602_ACCEL_ZOUT_L,
    ICM20602_GYRO_XOUT_H,  ICM20602_GYRO_XOUT_L,
    ICM20602_GYRO_YOUT_H,  ICM20602_GYRO_YOUT_L,
    ICM20602_GYRO_ZOUT_H,  ICM20602_GYRO_ZOUT_L
};

static void ICM20602_Init_Chip(void);
static void ICM20602_WriteReg(uint8_t RegAddress, uint8_t ucData);

int ICM20602_Init_Bare(void)
{
    ICM20602_Init_Chip();
    return 0;
}

void ICM20602_PreInit_PacketBuffers(uint8_t* ping_buf, uint8_t* pong_buf)
{
    SerialImuPacket_t* p_packets;
    
    // 初始化 Ping/Pong 缓冲区的包头和序号
    // 注意：现在我们要处理 12 个包
    uint8_t total_packets = sizeof(g_ucSerialMap);

    p_packets = (SerialImuPacket_t*)ping_buf;
    for (uint8_t i = 0; i < total_packets; i++) {
        p_packets[i].ucHead[0] = 0xA5;
        p_packets[i].ucHead[1] = 0x5A;
        p_packets[i].ucSerialNumber = g_ucSerialMap[i];
    }

    p_packets = (SerialImuPacket_t*)pong_buf;
    for (uint8_t i = 0; i < total_packets; i++) {
        p_packets[i].ucHead[0] = 0xA5;
        p_packets[i].ucHead[1] = 0x5A;
        p_packets[i].ucSerialNumber = g_ucSerialMap[i];
    }
}

int ICM20602_ReadBurst_Bare(SerialImuPacket_t* pPacketBuffer)
{
    // =========================================================================
    // 升级点 3: 循环读取 12 次，获取所有轴的数据
    // =========================================================================
    for (uint8_t i = 0; i < 12; i++) {
        I2C_Start();
        I2C_SendByte(ICM20602_I2C_ADDRESS, g_ucAck);
        I2C_SendByte(g_ucRegisterMap[i], g_ucAck);
        I2C_Start();
        I2C_SendByte(ICM20602_I2C_ADDRESS | 0x01, g_ucAck);
        I2C_ReceiveByte(pPacketBuffer[i].ucData, 0); // NACK
        I2C_Stop();
    }
    return 0;
}

static void ICM20602_Init_Chip(void)
{
    I2C_Config();
    ICM20602_WriteReg(ICM20602_PWR_MGMT_1, 0x01);
    ICM20602_WriteReg(ICM20602_PWR_MGMT_2, 0x00);
    ICM20602_WriteReg(ICM20602_SMPLRT_DIV, 0x07);
    ICM20602_WriteReg(ICM20602_GYRO_CONFIG, 0x18);
    ICM20602_WriteReg(ICM20602_ACCEL_CONFIG, 0x18);
    ICM20602_WriteReg(ICM20602_ACCEL_CONFIG2, 0x06);

    // Sensor 1 修复逻辑 (保持之前的代码)
    for(volatile uint32_t i = 0; i < 720000; i++);
    uint8_t check_buf[I2C_NUM];
    I2C_Start();
    I2C_SendByte(ICM20602_I2C_ADDRESS, g_ucAck);
    I2C_SendByte(ICM20602_ACCEL_CONFIG, g_ucAck);
    I2C_Start();
    I2C_SendByte(ICM20602_I2C_ADDRESS | 0x01, g_ucAck);
    I2C_ReceiveByte(check_buf, 0); 
    I2C_Stop();
    if (check_buf[0] != 0x18) {
        ICM20602_WriteReg(ICM20602_PWR_MGMT_1, 0x01);
        for(volatile uint32_t k = 0; k < 360000; k++);
        ICM20602_WriteReg(ICM20602_ACCEL_CONFIG, 0x18);
        ICM20602_WriteReg(ICM20602_GYRO_CONFIG, 0x18);
    }
}

static void ICM20602_WriteReg(uint8_t RegAddress, uint8_t ucData)
{
    I2C_Start();
    I2C_SendByte(ICM20602_I2C_ADDRESS, g_ucAck);
    I2C_SendByte(RegAddress, g_ucAck);
    I2C_SendByte(ucData, g_ucAck);
    I2C_Stop();
}