#include "MyI2C.h"
#include "Delay.h"
#include "GpioPin.h"
#include "Serial.h"
#include <cstring>

template<size_t... Is>
struct index_sequence {};

template<size_t N, size_t... Is>
struct make_index_sequence_impl : make_index_sequence_impl<N - 1, N - 1, Is...> {};

template<size_t... Is>
struct make_index_sequence_impl<0, Is...> {
    using type = index_sequence<Is...>;
};

template<size_t N>
using make_index_sequence = typename make_index_sequence_impl<N>::type;

template<typename... Ts> struct PinList {};

using SensorPins = PinList<
    PB<0>, PB<1>, PB<2>, PB<5>, PB<6>, PB<7>, PB<8>, PB<9>,    // Ch 0-7
    PF<7>, PF<8>, PB<12>, PB<13>, PB<14>, PB<15>, PC<0>, PC<1>, // Ch 8-15
    PC<2>, PC<3>, PC<4>, PC<5>, PC<6>, PC<7>, PC<8>, PC<9>,    // Ch 16-23 (包含 PC4/Ch18)
    PF<9>, PF<10>, PF<11>, PF<12>, PD<3>, PD<4>, PD<5>, PD<7>, // Ch 24-31
    PD<6>, PD<8>, PD<9>, PD<10>, PD<11>, PD<12>, PD<13>, PD<14>,// Ch 32-39
    PD<15>, PE<0>, PE<1>, PE<2>, PE<3>, PE<4>, PE<5>, PE<6>,   // Ch 40-47
    PE<7>, PE<8>, PE<9>, PE<10>, PE<11>, PE<12>, PE<13>, PE<14>,// Ch 48-55
    PE<15>, PF<0>, PF<1>, PF<2>, PF<3>, PF<4>, PF<5>, PF<6>    // Ch 56-63 (包含 PF6/Ch63)
>;

using SCL_GroupA = PinList<PA<4>, PA<5>, PA<6>, PA<7>>;
using SCL_GroupG = PinList<PG<0>, PG<1>, PG<2>, PG<3>>;

// ============================================================================
// 编译期遍历与掩码计算
// ============================================================================
template<typename... Pins, typename Func, size_t... Is>
__attribute__((always_inline))
void for_each_pin_impl(PinList<Pins...>, Func& f, index_sequence<Is...>) {
    (f.template operator()<Is, Pins>(), ...);
}

// 对外接口
template<typename... Pins, typename Func>
__attribute__((always_inline))
void for_each_pin(PinList<Pins...>, Func f) {
    for_each_pin_impl(PinList<Pins...>{}, f, make_index_sequence<sizeof...(Pins)>{});
}

// === 编译期掩码计算 ===
template<uint32_t TargetPortAddr, typename... Pins>
constexpr uint16_t get_port_mask(PinList<Pins...>) {
    uint16_t mask = 0;
    ((mask |= (Pins::BaseAddr == TargetPortAddr ? Pins::PinMask : 0)), ...);
    return mask;
}

constexpr uint16_t MASK_SDA_B = get_port_mask<GPIOB_BASE>(SensorPins{});
constexpr uint16_t MASK_SDA_C = get_port_mask<GPIOC_BASE>(SensorPins{});
constexpr uint16_t MASK_SDA_D = get_port_mask<GPIOD_BASE>(SensorPins{});
constexpr uint16_t MASK_SDA_E = get_port_mask<GPIOE_BASE>(SensorPins{});
constexpr uint16_t MASK_SDA_F = get_port_mask<GPIOF_BASE>(SensorPins{});

constexpr uint16_t MASK_SCL_A = get_port_mask<GPIOA_BASE>(SCL_GroupA{});
constexpr uint16_t MASK_SCL_G = get_port_mask<GPIOG_BASE>(SCL_GroupG{});

// === 时序参数 ===
constexpr uint16_t I2C_DELAY_HIGH_TICKS=50;
constexpr uint16_t I2C_DELAY_LOW_TICKS=75;

static inline void SCL_High() {
    GPIOA->BSRR = MASK_SCL_A; 
    GPIOG->BSRR = MASK_SCL_G;
    Delay_ticks(I2C_DELAY_HIGH_TICKS);
}

static inline void SCL_Low() {
    GPIOA->BRR = MASK_SCL_A; 
    GPIOG->BRR = MASK_SCL_G;
    Delay_ticks(I2C_DELAY_LOW_TICKS);
}

static inline void SDA_High() {
    GPIOB->BSRR = MASK_SDA_B; 
    GPIOC->BSRR = MASK_SDA_C;
    GPIOD->BSRR = MASK_SDA_D; 
    GPIOE->BSRR = MASK_SDA_E; 
    GPIOF->BSRR = MASK_SDA_F;
    Delay_ticks(I2C_DELAY_HIGH_TICKS);
}

static inline void SDA_Low() {
    GPIOB->BRR = MASK_SDA_B; 
    GPIOC->BRR = MASK_SDA_C;
    GPIOD->BRR = MASK_SDA_D; 
    GPIOE->BRR = MASK_SDA_E; 
    GPIOF->BRR = MASK_SDA_F;
    Delay_ticks(I2C_DELAY_LOW_TICKS);
}

struct BitReader {
    uint8_t* data;
    uint8_t  mask;
    // 寄存器缓存
    uint16_t vB, vC, vD, vE, vF;

    BitReader(uint8_t* d, uint8_t m) : data(d), mask(m) {
        // 在 SCL 高电平时，一次性快速读取所有 IDR
        vB = GPIOB->IDR; vC = GPIOC->IDR; 
        vD = GPIOD->IDR; vE = GPIOE->IDR; 
        vF = GPIOF->IDR;
    }

    template<size_t Index, typename Pin>
    __attribute__((always_inline))
    void operator()() {  
        uint16_t portVal = 0;
        if constexpr (Pin::BaseAddr == GPIOB_BASE) portVal = vB;
        else if constexpr (Pin::BaseAddr == GPIOC_BASE) portVal = vC;
        else if constexpr (Pin::BaseAddr == GPIOD_BASE) portVal = vD;
        else if constexpr (Pin::BaseAddr == GPIOE_BASE) portVal = vE;
        else if constexpr (Pin::BaseAddr == GPIOF_BASE) portVal = vF;

        if (portVal & Pin::PinMask) data[Index] |= mask;
    }
};

struct AckReader {
    uint8_t* ack;
    uint16_t vB, vC, vD, vE, vF;

    AckReader(uint8_t* a) : ack(a) {
        vB = GPIOB->IDR; vC = GPIOC->IDR; 
        vD = GPIOD->IDR; vE = GPIOE->IDR; 
        vF = GPIOF->IDR;
    }

    template<size_t Index, typename Pin>
    __attribute__((always_inline))
    void operator()() {
        uint16_t portVal = 0;
        if constexpr (Pin::BaseAddr == GPIOB_BASE) portVal = vB;
        else if constexpr (Pin::BaseAddr == GPIOC_BASE) portVal = vC;
        else if constexpr (Pin::BaseAddr == GPIOD_BASE) portVal = vD;
        else if constexpr (Pin::BaseAddr == GPIOE_BASE) portVal = vE;
        else if constexpr (Pin::BaseAddr == GPIOF_BASE) portVal = vF;
        
        // 读取 ACK 位：如果从机拉低 SDA (ACK)，则为 0；否则为 1 (NACK)
        ack[Index] = !!(portVal & Pin::PinMask);
    }
};

struct PinInit {
    template<size_t I, typename P> void operator()() {
        GPIO_InitTypeDef cfg;
        cfg.GPIO_Pin = P::PinMask;
        cfg.GPIO_Mode = GPIO_Mode_Out_OD;
        cfg.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(P::Port(), &cfg);
        P::set();
    }
};

void ParallelI2CManager::init() {
    // 关闭冲突外设
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | 
                           RCC_APB1Periph_I2C1 | RCC_APB1Periph_CAN1, DISABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, DISABLE);
    FSMC_Bank1->BTCR[4] = 0x00000000; 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, DISABLE);
    
    // 开启 GPIO 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | 
                           RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | 
                           RCC_APB2Periph_GPIOG, ENABLE);
    
    // 禁用外部 SRAM (NE3)
    GPIO_InitTypeDef sram_cs;
    sram_cs.GPIO_Pin = GPIO_Pin_10;
    sram_cs.GPIO_Mode = GPIO_Mode_Out_PP;
    sram_cs.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOG, &sram_cs);
    GPIO_SetBits(GPIOG, GPIO_Pin_10);

    // 初始化所有通道引脚
    for_each_pin(SCL_GroupA{}, PinInit{});
    for_each_pin(SCL_GroupG{}, PinInit{});
    for_each_pin(SensorPins{}, PinInit{});
    
    clearError();
}

void ParallelI2CManager::start() {
    SDA_High(); Delay_ticks(I2C_DELAY_HIGH_TICKS);
    SCL_High(); Delay_ticks(I2C_DELAY_HIGH_TICKS);
    SDA_Low();  Delay_ticks(I2C_DELAY_LOW_TICKS);
    SCL_Low();
}

void ParallelI2CManager::stop() {
    SDA_Low();  Delay_ticks(I2C_DELAY_LOW_TICKS);
    SCL_High(); Delay_ticks(I2C_DELAY_HIGH_TICKS);
    SDA_High();
}

void ParallelI2CManager::sendByte(uint8_t byte, uint8_t* pAckBuffer) {
	ScopedIrqLock lock;
    // 发送 8 位
    for (uint8_t i = 0; i < 8; i++) {
        if ((byte >> (7 - i)) & 0x01) SDA_High(); else SDA_Low();
        SCL_High(); SCL_Low();
    }
    
    // 释放总线，准备读取 ACK
    SDA_High(); 
    SCL_High();
    
    uint8_t ack_local[I2C_NUM] = {0};
    AckReader reader(ack_local);
    for_each_pin(SensorPins{}, reader);
    
    SCL_Low();

    // 错误检测 (ACK=0 表示成功, ACK=1 表示 NACK)
    for (int i = 0; i < I2C_NUM; i++) {
        if (ack_local[i] != 0) { 
            error_detected_ = true;
            break;
        }
    }
    
    if (pAckBuffer) {
        memcpy(pAckBuffer, ack_local, I2C_NUM);
    }
}

void ParallelI2CManager::receiveByte(uint8_t* pRxBuffer, bool sendAck) {
	ScopedIrqLock lock;
    SDA_High(); // 释放总线准备读取
    memset(pRxBuffer, 0, I2C_NUM);

    for (uint8_t j = 0; j < 8; j++) {
        SCL_High();
        BitReader reader(pRxBuffer, (0x80 >> j));
        for_each_pin(SensorPins{}, reader);
        SCL_Low();
    }
    // 发送 ACK/NACK
    if (sendAck) SDA_Low(); else SDA_High();
    SCL_High(); SCL_Low();
    SDA_High(); // 释放
}
