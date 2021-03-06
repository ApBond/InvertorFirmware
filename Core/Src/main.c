#include "main.h"

static uint16_t communicateProcessTimer=0;
static uint16_t testTimer=0;

void TIM6_DAC1_IRQHandler(void)
{
    if(TIM6->SR & TIM_SR_UIF)
    {
        TIM6->SR &=~TIM_SR_UIF;
        communicateProcessTimer++;
        testTimer++;
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
    uint16_t filter1[]={BROADCAST_ID,INDIVIDUAL_ID1,ERROR_CHANNEL_ID,5};
    uint16_t filter2[]={ROS_CONTROL_ID,SET_VECTOR_ID,INDIVIDUAL_ID2,6};
    while(RccClockInit()!=READY);
    delayInit();
    canInit(0x011c0008);
    canConfigReciveFIFO(1,0,ID,&filter1);
    canConfigReciveFIFO(1,1,ID,&filter2);
    hallTimInit();
    ADCInit();
    PWMInit();
    speedLoopTimerInit();
    timer6Init();

    tim16Init();
    I2CInit();
    delay_ms(10);
    //calibration();
    //DMAInit();
    //motorStart();

    while(1)
    {
        if(communicateProcessTimer>=COMMUNICATE_PERIOD)
        {
            sendDiagnosticData();
            communicateProcessTimer=0;
        }
        /*if(testTimer>=1000)
        {
            //I2C_Receiver(0x48,2);
            //testTimer=0;
        }*/
        userCommunicationProcess();
    }
}