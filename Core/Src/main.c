#include "main.h"


int main(void)
{
    uint8_t i;
    uint8_t data[]={1,2,3,4,5,6,7,8};
    can_recive_message_t* reciveData;
    while(RccClockInit()!=READY);
    delayInit();
    canInit(0x011c0008);
    RCC->AHBENR|=RCC_AHBENR_GPIOBEN;
    GPIOB->MODER=0;
    GPIOB->MODER|=GPIO_MODER_MODER3_0;
    GPIOB->BSRR=GPIO_BSRR_BS_3;
    while(1)
    {
        //canWrite(data,2,12);
        reciveData=canRead();
        if(reciveData!=0)
            canWrite(&reciveData->data,reciveData->dataLen,reciveData->messageId);
        delay_ms(10);
        i++;
    }
}