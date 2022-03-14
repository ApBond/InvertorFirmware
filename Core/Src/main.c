#include "main.h"

extern int16_t currentA;
extern int16_t currentC;
extern d_q_t Idq;
extern d_q_t Us;
uint16_t temp;
int main(void)
{
    uint8_t i;
    uint8_t data[8];
    uint16_t filter[]={0x1,0x2,0x3,0x4};
    can_recive_message_t* reciveData;
    int16_t speed;
    int16_t Id;
    int16_t Iq;
    int16_t Ud;
    int16_t Uq;
    while(RccClockInit()!=READY);
    delayInit();
    canInit(0x011c0008);
    canConfigReciveFIFO(1,0,ID,&filter);
    hallTimInit();
    ADCInit();
    PWMInit();
    speedLoopTimerInit();
    temp=sizeof(float);
    GPIOB->MODER|= GPIO_MODER_MODER5_0;
    while(1)
    {
        speed=(int16_t)(getSpeed());
        Id=(int16_t)(getReferenceTorque()*1000);
        Iq=(int16_t)(Idq.q*1000);
        Ud=(int16_t)(Us.d*1000);
        Uq=(int16_t)(Us.q*1000);
        data[0]=speed & 0xFF;
        data[1]=speed >> 8;
        //data[0]=Ud & 0xFF;
        //data[1]=Ud >> 8;
        data[2]=Uq & 0xFF;
        data[3]=Uq >> 8;
        data[4]=Id & 0xFF;
        data[5]=Id>>8;
        data[6]=Iq & 0xFF;
        data[7]=Iq >> 8;
        canWrite(data,8,10);
        reciveData=canRead();
        if(reciveData!=0)
        {
            if(reciveData->messageId==0x01 && reciveData->data[0]==1)
            {
                PWMStart();
                startSpeedLoop();
            }
            if(reciveData->messageId==0x01 && reciveData->data[0]==0)
            {
                PWMStop();  
                stopSpeedLoop(); 
            }
        }
        //ADC1->CR |= ADC_CR_JADSTART;
        //ADC2->CR |= ADC_CR_JADSTART;
        //canWrite(&reciveData->data,reciveData->dataLen,reciveData->messageId);
        delay_ms(2);
    }
}