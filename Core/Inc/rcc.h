#ifndef __RCC_H
#define __RCC_H

#include "main.h"

typedef enum
{
    READY,
    HSE_NOT_ENABLE,
    PLL_NOT_ENABLE
}RCC_STATUS_t;

#define SYS_CORE_FREQUENSY 72000000

static volatile uint32_t TimingDelay;

RCC_STATUS_t RccClockInit(void);
uint32_t delayInit(void);
void delay_ms(volatile uint32_t nTime);
void delay_us(volatile uint32_t nTime);
void timingDelayDecrement(void);



#endif