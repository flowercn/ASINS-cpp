/**
 * @file Serial.cpp
 * @brief 串口通信及DMA乒乓缓冲机制实现
 * @author Gemini
 * @date 2025-08-30
 */

#include "Serial.h"

//==================================================================================================
// 模块级私有定义 (Module-Private Definitions)
//==================================================================================================

// 通过宏定义选择使用的USART端口，方便在USART2和USART3之间切换
//#define USE_USART3_FOR_422

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
    // --- USART2 (默认) 配置 ---
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

//! DMA乒乓缓冲区的实体
static uint8_t s_ping_buffer[DMA_BUFFER_SIZE];
static uint8_t s_pong_buffer[DMA_BUFFER_SIZE];

// --- 模块级变量 ---
uint8_t* p_ping_buffer = s_ping_buffer;
uint8_t* p_pong_buffer = s_pong_buffer;

volatile uint8_t* s_dma_current_buffer = NULL;
volatile uint8_t* s_dma_next_buffer = NULL;

//==================================================================================================
// 私有函数声明 (Private Function Declarations)
//==================================================================================================

static void USART_Config(uint32_t bound);
static void USART_DMA_Tx_Config(void);

//==================================================================================================
// 公共函数实现 (Public Function Implementations)
//==================================================================================================

/**
 * @brief 初始化串口及DMA
 */
void Serial_Init(uint32_t ulBaudrate)
{
    USART_Config(ulBaudrate);
    USART_DMA_Tx_Config();
}

/**
 * @brief 发送一个数据缓冲区
 */
int Serial_SendBuffer(uint8_t* pBuffer)
{
    __disable_irq(); // 进入临界区

    if (s_dma_current_buffer == NULL) // 如果DMA空闲
    {
        s_dma_current_buffer = pBuffer;
        DMA_Cmd(SERIAL_DMA_CHANNEL, DISABLE);
        SERIAL_DMA_CHANNEL->CMAR = (uint32_t)pBuffer; // 设置内存地址
        DMA_SetCurrDataCounter(SERIAL_DMA_CHANNEL, DMA_BUFFER_SIZE); // 设置传输大小
        DMA_Cmd(SERIAL_DMA_CHANNEL, ENABLE); // 启动DMA
    }
    else if (s_dma_next_buffer == NULL) // 如果DMA忙，但下一个缓冲区空闲
    {
        s_dma_next_buffer = pBuffer; // 将缓冲区置于等待队列
    }
    else // 如果两个缓冲区都在使用中
    {
        __enable_irq(); // 退出临界区
        return -1; // 返回失败，表示缓冲区已满
    }

    __enable_irq(); // 退出临界区
    return 0; // 成功加入发送队列
}

//==================================================================================================
// 私有函数实现 (Private Function Implementations)
//==================================================================================================

/**
 * @brief 配置USART硬件参数
 * @param[in] bound 要设置的波特率
 */
static void USART_Config(uint32_t bound)
{
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

    // 使能USART的DMA发送请求
    USART_DMACmd(SERIAL_USART_PERIPH, USART_DMAReq_Tx, ENABLE);
    // 使能USART
    USART_Cmd(SERIAL_USART_PERIPH, ENABLE);
}

/**
 * @brief 配置USART发送所使用的DMA通道
 */
static void USART_DMA_Tx_Config(void)
{
    DMA_InitTypeDef DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 使能DMA时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 配置DMA通道参数
    DMA_DeInit(SERIAL_DMA_CHANNEL);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&SERIAL_USART_PERIPH->DR; // 外设地址
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)s_ping_buffer; // 内存地址 (初始指向ping)
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

//==================================================================================================
// 中断服务函数 (Interrupt Service Routines)
//==================================================================================================

// C++ 中断处理函数必须用 extern "C" 包装，防止 name mangling
extern "C" {

/**
 * @brief DMA传输完成中断服务函数
 * @details 当一个DMA缓冲区发送完成后，此函数被调用。
 *          它会检查是否有下一个缓冲区在等待发送（乒乓缓冲机制），
 *          如果有，则启动下一次DMA传输；如果没有，则将DMA状态设置为空闲。
 */
void SERIAL_DMA_IRQHandler(void)
{
    if (DMA_GetITStatus(SERIAL_DMA_IT_GL_FLAG) != RESET)
    {
        DMA_ClearITPendingBit(SERIAL_DMA_IT_GL_FLAG); // 清除中断标志
        DMA_Cmd(SERIAL_DMA_CHANNEL, DISABLE); // 停止DMA，准备切换

        if (s_dma_next_buffer != NULL) // 检查是否有等待中的缓冲区
        {
            // 切换缓冲区指针
            s_dma_current_buffer = s_dma_next_buffer;
            s_dma_next_buffer = NULL;
            
            // 配置并启动下一次DMA传输
            SERIAL_DMA_CHANNEL->CMAR = (uint32_t)s_dma_current_buffer;
            DMA_SetCurrDataCounter(SERIAL_DMA_CHANNEL, DMA_BUFFER_SIZE);
            DMA_Cmd(SERIAL_DMA_CHANNEL, ENABLE);
        }
        else
        {
            s_dma_current_buffer = NULL; // 没有等待的缓冲区，DMA变为空闲
        }
    }
}

} // extern "C"
