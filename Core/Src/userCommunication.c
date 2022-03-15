#include "userCommunication.h"

void sendDiagnosticData(void)
{
    uint8_t data[8];
    int16_t speed;
    int16_t Id;
    int16_t Iq;
    int16_t Ud;
    int16_t Uq;
    struct d_q_t Us;
    struct d_q_t Idq;
    Us=getControllImpact();
    Idq=getCurrentIdq();
    speed=(int16_t)(getSpeed());
    Id=(int16_t)(getReferenceTorque()*1000);
    Iq=(int16_t)(Idq.q*1000);
    Ud=(int16_t)(Us.d*1000);
    Uq=(int16_t)(Us.q*1000);
    data[0]=speed & 0xFF;
    data[1]=speed >> 8;
    data[2]=Uq & 0xFF;
    data[3]=Uq >> 8;
    data[4]=Id & 0xFF;
    data[5]=Id>>8;
    data[6]=Iq & 0xFF;
    data[7]=Iq >> 8;
    canWrite(data,8,10);
}