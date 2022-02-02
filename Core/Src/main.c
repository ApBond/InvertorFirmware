#include "main.h"


int main(void)
{
    uint8_t i;
    uint8_t data[]={0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8};
    uint16_t filter[]={0x1,0x2,0x3,0x4};
    can_recive_message_t* reciveData;
    uint8_t speed;
    while(RccClockInit()!=READY);
    delayInit();
    canInit(0x011c0008);
    canConfigReciveFIFO(1,0,ID,&filter);
    hallTimInit();
    while(1)
    {
        speed=(uint8_t)getSpeed();
        canWrite(&speed,1,10);
        /*reciveData=canRead();
        if(reciveData!=0)
            canWrite(&reciveData->data,reciveData->dataLen,reciveData->messageId);*/
        delay_ms(200);
        i++;
    }
}