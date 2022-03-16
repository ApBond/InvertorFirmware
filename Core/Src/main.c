#include "main.h"

static uint16_t communicateProcessTimer=0;

void TIM6_DAC1_IRQHandler(void)
{
    if(TIM6->SR & TIM_SR_UIF)
    {
        TIM6->SR &=~TIM_SR_UIF;
        communicateProcessTimer++;
    }
}

void timer6Init(void)
{ 
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 7200-1;
    TIM6->ARR = 10-1;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM6_DAC1_IRQn);
}

int main(void)
{
    uint8_t i;
    uint16_t filter[]={BROADCAST_ID,SET_VECTOR_ID,INDIVIDUAL_ID1,INDIVIDUAL_ID2};
    while(RccClockInit()!=READY);
    delayInit();
    canInit(0x011c0008);
    canConfigReciveFIFO(1,0,ID,&filter);
    hallTimInit();
    ADCInit();
    PWMInit();
    speedLoopTimerInit();
    timer6Init();
    while(1)
    {
        if(communicateProcessTimer>=COMMUNICATE_PERIOD)
        {
            sendDiagnosticData();
            communicateProcessTimer=0;
        }
        userCommunicationProcess();
    }
}