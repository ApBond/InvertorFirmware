#include "main.h"

extern int16_t currentA;
extern int16_t currentC;
extern d_q_t Idq;

uint16_t temp;
int main(void)
{
    uint8_t i;
    uint8_t data[5];
    uint16_t filter[]={0x1,0x2,0x3,0x4};
    can_recive_message_t* reciveData;
    uint8_t speed;
    int16_t Id;
    int16_t Iq;
    while(RccClockInit()!=READY);
    delayInit();
    canInit(0x011c0008);
    canConfigReciveFIFO(1,0,ID,&filter);
    hallTimInit();
    ADCInit();
    PWMInit();
    temp=sizeof(float);
    
    //PWMStart();
    while(1)
    {
        //speed=(uint8_t)getSpeed();
        Id=(int16_t)(Idq.d*1000);
        Iq=(int16_t)(Idq.q*1000);
        data[0]=(uint8_t)getSpeed();
        /*data[1]=currentA & 0xF;
        data[2]=currentA>>8;
        data[3]=currentC & 0xF;
        data[4]=currentC >> 8;*/
        data[1]=Id & 0xF;
        data[2]=Id>>8;
        data[3]=Iq & 0xF;
        data[4]=Iq >> 8;
        canWrite(data,5,10);
        reciveData=canRead();
        if(reciveData!=0)
        {
            if(reciveData->messageId==0x01 && reciveData->data[0]==1)
                PWMStart();
            if(reciveData->messageId==0x01 && reciveData->data[0]==0)
                PWMStop();   
        }
        //ADC1->CR |= ADC_CR_JADSTART;
        //ADC2->CR |= ADC_CR_JADSTART;
        //canWrite(&reciveData->data,reciveData->dataLen,reciveData->messageId);
        delay_ms(2);
    }
}