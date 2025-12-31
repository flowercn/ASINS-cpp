/**
 * @file Serial.cpp
 * @brief 串口通信及DMA乒乓缓冲机制实现
 * @author dengyihang
 * @date 2025-08-30/2025-12-31
 */

#include "Serial.h"
#include "cpp_main.h"

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

uint8_t SerialManager::s_buffer_active[DMA_BUFFER_SIZE];
uint8_t SerialManager::s_buffer_shadow[DMA_BUFFER_SIZE];

void SerialManager::init(uint32_t baudrate) {
    configHardware(baudrate);
    configDma();
}

bool SerialManager::transmit(uint8_t* pBuffer){
	ScopedIrqLock lock;
	if (current_tx_ptr == nullptr){
		current_tx_ptr =pBuffer;
		SERIAL_DMA_CHANNEL->CMAR = reinterpret_cast<uint32_t>(pBuffer);
        SERIAL_DMA_CHANNEL->CNDTR = DMA_BUFFER_SIZE;
        DMA_Cmd(SERIAL_DMA_CHANNEL, ENABLE);
        return true;
	}
	else if (next_tx_ptr == nullptr) {
        next_tx_ptr = pBuffer;
        return true;
    }
	return false;
}

uint8_t SerialManager::getCommand(){
	uint8_t cmd = received_cmd;
	received_cmd = 0;
	return cmd;
}

void SerialManager::handleDmaIsr() {
	if(DMA_GetITStatus(SERIAL_DMA_IT_GL_FLAG) != RESET) {
		DMA_ClearITPendingBit(SERIAL_DMA_IT_GL_FLAG);
        DMA_Cmd(SERIAL_DMA_CHANNEL, DISABLE);
		
		if (next_tx_ptr != nullptr) {
			current_tx_ptr =next_tx_ptr;
			next_tx_ptr = nullptr;
			
			SERIAL_DMA_CHANNEL->CMAR = reinterpret_cast<uint32_t>(current_tx_ptr);
            SERIAL_DMA_CHANNEL->CNDTR = DMA_BUFFER_SIZE;
			DMA_Cmd(SERIAL_DMA_CHANNEL, ENABLE);
		} 
		else {
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
    // ... 原 USART_Config 逻辑 ...
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
    // ... 原 USART_DMA_Tx_Config 逻辑 ...
	    DMA_InitTypeDef DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 使能DMA时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 配置DMA通道参数
    DMA_DeInit(SERIAL_DMA_CHANNEL);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&SERIAL_USART_PERIPH->DR; // 外设地址
    DMA_InitStruct.DMA_MemoryBaseAddr = reinterpret_cast<uint32_t>(s_buffer_active);// 内存地址 (初始指向active)
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST; // 方向：内存到外设
    DMA_InitStruct.DMA_BufferSize = DMA_BUFFER_SIZE; // 缓冲区大小
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 外设地址不增
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable; // 内存地址递增
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal; // 普通模式（非循环）
    DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh; // 优先级
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable; // 非内存到内存模式
    DMA_Init(SERIAL_DMA_CHANNEL, &DMA_InitStruct);

    // 使能DMA传输完成中断
    DMA_ITConfig(SERIAL_DMA_CHANNEL, DMA_IT_TC, ENABLE);

    // 配置DMA中断的NVIC (嵌套向量中断控制器)
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
