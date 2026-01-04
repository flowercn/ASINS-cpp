#include "Serial.h"

#ifdef USE_USART3_FOR_422
    // --- USART3 (RS-422) 配置 ---
    #define SERIAL_USART_PERIPH         USART3
    #define SERIAL_USART_RCC            RCC_APB1Periph_USART3
    #define SERIAL_GPIO_PERIPH          GPIOB
    #define SERIAL_GPIO_RCC             RCC_APB2Periph_GPIOB
    #define SERIAL_TX_PIN               GPIO_Pin_10
    #define SERIAL_RX_PIN               GPIO_Pin_11
    #define SERIAL_DMA_CHANNEL          DMA1_Channel2
    #define SERIAL_DMA_IT_GL_FLAG       DMA1_IT_GL2
    #define SERIAL_DMA_IRQn             DMA1_Channel2_IRQn
    #define SERIAL_DMA_IRQHandler       DMA1_Channel2_IRQHandler
#else
    // --- USART2 (CH340串口) 配置 ---
    #define SERIAL_USART_PERIPH         USART2
    #define SERIAL_USART_RCC            RCC_APB1Periph_USART2
    #define SERIAL_GPIO_PERIPH          GPIOA
    #define SERIAL_GPIO_RCC             RCC_APB2Periph_GPIOA
    #define SERIAL_TX_PIN               GPIO_Pin_2
    #define SERIAL_RX_PIN               GPIO_Pin_3
    #define SERIAL_DMA_CHANNEL          DMA1_Channel7
    #define SERIAL_DMA_IT_GL_FLAG       DMA1_IT_GL7
    #define SERIAL_DMA_IRQn             DMA1_Channel7_IRQn
    #define SERIAL_DMA_IRQHandler       DMA1_Channel7_IRQHandler
#endif

uint8_t SerialManager::s_buffer_active[MAX_FRAME_SIZE];
uint8_t SerialManager::s_buffer_shadow[MAX_FRAME_SIZE];

void SerialManager::init(uint32_t baudrate) {
    configHardware(baudrate);
    configDma();
}

bool SerialManager::isBufferBusy(uint8_t* pBuf) {
    ScopedIrqLock lock; // 加锁
    return (current_tx_ptr == pBuf) || (next_tx_ptr == pBuf);
}

bool SerialManager::transmit(uint8_t* pBuffer, size_t length) {
    ScopedIrqLock lock;
    
    if (current_tx_ptr == nullptr) {
        // 空闲发送
        current_tx_ptr = pBuffer;
        SERIAL_DMA_CHANNEL->CMAR = reinterpret_cast<uint32_t>(pBuffer);
        SERIAL_DMA_CHANNEL->CNDTR = length; // 写入实际长度
        DMA_Cmd(SERIAL_DMA_CHANNEL, ENABLE);
        return true;
    }
    else if (next_tx_ptr == nullptr) {
        // 忙等待
        next_tx_ptr = pBuffer;
        next_tx_length = length; // 记录等待发送的长度
        return true;
    }
    return false; // 队列满
}

SerialCommand SerialManager::getCommand(){
	uint8_t cmd = received_cmd;
	received_cmd = 0;
	return static_cast<SerialCommand>(cmd);
}

void SerialManager::handleDmaIsr() {
    if (DMA_GetITStatus(SERIAL_DMA_IT_GL_FLAG) != RESET) {
        DMA_ClearITPendingBit(SERIAL_DMA_IT_GL_FLAG);
        DMA_Cmd(SERIAL_DMA_CHANNEL, DISABLE);
        if (next_tx_ptr != nullptr) {
            // 切换缓冲区
            current_tx_ptr = next_tx_ptr;
            SERIAL_DMA_CHANNEL->CMAR = reinterpret_cast<uint32_t>(current_tx_ptr);
            SERIAL_DMA_CHANNEL->CNDTR = next_tx_length; // 使用记录的长度
            next_tx_ptr = nullptr;
            DMA_Cmd(SERIAL_DMA_CHANNEL, ENABLE);
        } else {
            current_tx_ptr = nullptr;
        }
    }
}

void SerialManager::handleUartIsr() {
    if (USART_GetITStatus(SERIAL_USART_PERIPH, USART_IT_RXNE) != RESET) {
        received_cmd = static_cast<uint8_t>(USART_ReceiveData(SERIAL_USART_PERIPH));
    }
}

void SerialManager::configHardware(uint32_t bound) {
	GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    // 使能USART和GPIO的时钟
    RCC_APB1PeriphClockCmd(SERIAL_USART_RCC, ENABLE);
    RCC_APB2PeriphClockCmd(SERIAL_GPIO_RCC, ENABLE);

    // 配置GPIO引脚
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Pin = SERIAL_TX_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
    GPIO_Init(SERIAL_GPIO_PERIPH, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = SERIAL_RX_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
    GPIO_Init(SERIAL_GPIO_PERIPH, &GPIO_InitStruct);

    // 配置USART参数
    USART_InitStruct.USART_BaudRate = bound;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_Init(SERIAL_USART_PERIPH, &USART_InitStruct);
    
    USART_DMACmd(SERIAL_USART_PERIPH, USART_DMAReq_Tx, ENABLE);  // 使能USART的DMA发送请求
    USART_Cmd(SERIAL_USART_PERIPH, ENABLE);      // 使能USART
	USART_ITConfig(SERIAL_USART_PERIPH, USART_IT_RXNE, ENABLE);  // 使能接收中断 (RXNE)
	
	// 配置 RX 中断优先级
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = (SERIAL_USART_PERIPH == USART3) ? USART3_IRQn : USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3; // 优先级稍低一点
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    // 注意：最后使能串口中断
}

void SerialManager::configDma() {
    DMA_InitTypeDef DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 使能DMA时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 配置DMA通道参数
    DMA_DeInit(SERIAL_DMA_CHANNEL);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&SERIAL_USART_PERIPH->DR; // 外设地址
    DMA_InitStruct.DMA_MemoryBaseAddr = reinterpret_cast<uint32_t>(s_buffer_active); // 内存地址
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST; // 方向：内存到外设
    DMA_InitStruct.DMA_BufferSize = MAX_FRAME_SIZE; // 初始给个最大值
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 外设地址不增
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable; // 内存地址递增
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal; // 普通模式
    DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh; // 优先级
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable; // 非内存到内存
    DMA_Init(SERIAL_DMA_CHANNEL, &DMA_InitStruct);

    // 使能DMA传输完成中断
    DMA_ITConfig(SERIAL_DMA_CHANNEL, DMA_IT_TC, ENABLE);

    // 配置DMA中断的NVIC
    NVIC_InitStruct.NVIC_IRQChannel = SERIAL_DMA_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

extern "C" {
    void DMA1_Channel7_IRQHandler(void) {
        SerialManager::getInstance().handleDmaIsr();
    }

    void USART2_IRQHandler(void) {
        SerialManager::getInstance().handleUartIsr();
    }
}


