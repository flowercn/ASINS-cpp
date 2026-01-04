#include "ICM20602.h"
#include <cstring>

static uint8_t g_dummyAck[I2C_NUM];
static const uint8_t g_ucSerialMap[] = {
    0x01, 0x02, // Accel X High, Low
    0x03, 0x04, // Accel Y High, Low
    0x05, 0x06, // Accel Z High, Low
    0x07, 0x08, // Gyro X High, Low
    0x09, 0x0A, // Gyro Y High, Low
    0x0B, 0x0C  // Gyro Z High, Low
};

void Icm20602Manager::initPacketHeaders(uint8_t* ping_buf, uint8_t* pong_buf) {
    SerialImuPacket* p_packets;
    uint8_t total_packets = sizeof(g_ucSerialMap);

    auto fill = [&](uint8_t* buf) {
        p_packets = (SerialImuPacket*)buf;
        for (uint8_t i = 0; i < total_packets; i++) {
            p_packets[i].head[0] = 0xA5;
            p_packets[i].head[1] = 0x5A;
            p_packets[i].serialNumber = g_ucSerialMap[i];
        }
    };
    fill(ping_buf);
    fill(pong_buf);
}

void Icm20602Manager::writeReg(uint8_t reg, uint8_t data) {
    ParallelI2C.start();
    ParallelI2C.sendByte(ADDR_WRITE, g_dummyAck);
    ParallelI2C.sendByte(reg, g_dummyAck);
    ParallelI2C.sendByte(data, g_dummyAck);
    ParallelI2C.stop();
}

void Icm20602Manager::init() {
    ParallelI2C.init(); // 初始化并行总线

    writeReg(ICM20602_PWR_MGMT_1, 0x01);
    writeReg(ICM20602_PWR_MGMT_2, 0x00);
    writeReg(ICM20602_SMPLRT_DIV, 0x07);
    writeReg(ICM20602_GYRO_CONFIG, 0x18);
    writeReg(ICM20602_ACCEL_CONFIG, 0x18);
    writeReg(ICM20602_ACCEL_CONFIG2, 0x06);
}

void Icm20602Manager::readAllSensors(SerialImuPacket* pPacketBuffer) {
    // 1. 发送写地址 + 寄存器首地址
    ParallelI2C.start();
    ParallelI2C.sendByte(ADDR_WRITE, g_dummyAck);
    ParallelI2C.sendByte(ICM20602_ACCEL_XOUT_H, g_dummyAck);
    
    // 2. 切换到读模式
    ParallelI2C.start();
    ParallelI2C.sendByte(ADDR_READ, g_dummyAck);

    // 3. 读取加速度 (0~5)
    for (int i = 0; i < 6; i++) {
        ParallelI2C.receiveByte(pPacketBuffer[i].data, true); 
    }

    // 4. 丢弃温度
    uint8_t dummy[I2C_NUM];
    ParallelI2C.receiveByte(dummy, true);
    ParallelI2C.receiveByte(dummy, true);

    // 5. 读取陀螺仪前5字节 (6~10)
    for (int i = 6; i < 11; i++) {
        ParallelI2C.receiveByte(pPacketBuffer[i].data, true);
    }

    // 6. 读取最后一个字节 (11) + NACK
    ParallelI2C.receiveByte(pPacketBuffer[11].data, false); 

    ParallelI2C.stop();
}
