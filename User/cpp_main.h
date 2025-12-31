#pragma once

#include "stm32f10x.h"
#include "Serial.h"
#include "ICM20602.h"

/**
 * @brief RAII 中断锁
 */
class ScopedIrqLock {
public:
    ScopedIrqLock() { __disable_irq(); }
    ~ScopedIrqLock() { __enable_irq(); }
    ScopedIrqLock(const ScopedIrqLock&) = delete;
    ScopedIrqLock& operator=(const ScopedIrqLock&) = delete;
};

void cpp_entry(void);
