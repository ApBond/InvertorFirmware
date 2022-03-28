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

void sendErrorState(void)
{
    FOC_Error_t error;
    error=getErrorState();
    canWrite((uint8_t*)&error,1,ERROR_CHANNEL_ID);
}

void userCommunicationProcess(void)
{
    can_recive_message_t* reciveData;
    int16_t temp16;
    int32_t temp32;
    FOC_Error_t error;
    reciveData=canRead();
    if(reciveData!=0)
    {
        if(reciveData->messageId == BROADCAST_ID || reciveData->messageId == INDIVIDUAL_ID1)
        {
            switch (reciveData->data[0])
            {
            case ALL_RESET:
                setErrorState(NOT_ERROR);
                break;
            case START_STOP_DRIVE:
                if(reciveData->data[1]==MOTOR_START)
                    motorStart();
                else if(reciveData->data[1]==MOTOR_STOP)
                    motorStop();
                break;
            case CHANGE_MODE:
                if(reciveData->data[1]=TORQUE_MODE)
                    setControlMode(TORQUE_CONTROL);
                else if(reciveData->data[1]=SPEED_MODE)
                    setControlMode(SPEED_CONTROL);
                break;
            case CHANGE_SPEED:
                temp16 =reciveData->data[1]+((reciveData->data[2])<<8);
                setReferenceSpeed((float)temp16);
                break;
            case CHANGE_TORQUE:
                temp32 = (reciveData->data[1]) + (reciveData->data[2]<<8) + (reciveData->data[3]<<16) + (reciveData->data[4]<<24);
                setReferenceTorque((float)temp32/65535);
                break;
            case CHANGE_ANGLE:
                break;
            default:
                break;
            }
        }
        else if(reciveData->messageId == SET_VECTOR_ID)
        {
            rcCommand_t cmd;
            float refSpeed;
            float refAngle[4];
            cmd.V = (float)((int16_t)(reciveData->data[0] + (reciveData->data[1]<<8)))/2048;
            cmd.gam = (float)((int16_t)(reciveData->data[2] + (reciveData->data[3]<<8)))/2048;
            cmd.R = (float)(int32_t)((reciveData->data[4] + (reciveData->data[5]<<8) + (reciveData->data[6]<<16) + (reciveData->data[7]<<24)))/65535;
            kinematica(cmd,&refSpeed,refAngle);
            setServoAngle(refAngle);
            setReferenceSpeed(refSpeed);
            
        }
        else if(reciveData->messageId == ERROR_CHANNEL_ID)
        {
            error = reciveData->data[0];
            setErrorState(error);
        }
    }
}
