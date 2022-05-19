#include "userCommunication.h"

Communication_mode_t communicationMode = NO_DATA;
Motion_control_mode_t motionMode = PDU_CONTROL;

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
    abc_t Iabc;
    float odomSpeed;
    float odomAngle;
    uint16_t temp16;
    if(communicationMode==DEBUG)
    {
        Iabc=getCurrentIabc();
        Us=getControllImpact();
        Idq=getCurrentIdq();
        speed=(int16_t)(getSpeed());
        Id=(int16_t)(getReferenceTorque()*1000);
        Iq=(int16_t)(Idq.q*1000);
        //Id=(int16_t)(Iabc.a*1000);
        //Iq=(int16_t)(Iabc.c*1000);
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
        canWrite(data,8,INDIVIDUAL_ID1);
        data[0]=getErrorState();
        data[1]=getMotorState();
        canWrite(data,2,INDIVIDUAL_ID2);
    } else if(communicationMode==ODOM)
    {
        odomAngle = getAngle();
        odomSpeed = getSpeed();
        temp16 = (uint16_t)((float)odomSpeed * 2048);
        data[0]=temp16 & 0xFF;
        data[1]=temp16 >> 8;
        temp16 = (uint16_t)((float)odomAngle * 2048);
        data[2]=temp16 & 0xFF;
        data[3]=temp16 >> 8;
        canWrite(data,4,INDIVIDUAL_ID2);
    }

}

void sendErrorState(void)
{
    uint8_t data[8];
    FOC_Error_t error;
    error=getErrorState();
    data[0]=(uint8_t)error;
    data[1] = (uint8_t)INDIVIDUAL_ID1;
    canWrite(data,2,ERROR_CHANNEL_ID);
}

void userCommunicationProcess(void)
{
    can_recive_message_t* reciveData;
    int16_t temp16;
    int32_t temp32;
    FOC_Error_t error;
    float refAngleTemp[4];
    uint8_t data[8];
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
                {
                    setErrorState(NOT_ERROR);
                    motorStart();
                }
                else if(reciveData->data[1]==MOTOR_STOP)
                    motorStop();
                break;
            case CHANGE_MODE:
                if(reciveData->data[1]==TORQUE_MODE)
                    setControlMode(TORQUE_CONTROL);
                else if(reciveData->data[1]==SPEED_MODE)
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
                temp16=reciveData->data[1]+((reciveData->data[2])<<8);
                refAngleTemp[0]=(float)((float)temp16*0.017);
                refAngleTemp[1]=refAngleTemp[0];
                refAngleTemp[2]=refAngleTemp[0];
                refAngleTemp[3]=refAngleTemp[0];
                setServoAngle(refAngleTemp);
                break;
            case CHANGE_DEVICE_COMMUNICATION_MODE:
                communicationMode=reciveData->data[1];
                break;
            case CHANGE_MOTION_CONTROL_MODE:
                motionMode = reciveData->data[1];
                break;
            case CHANGE_TO_BOOT_MODE:
                motorStop();
                flashUnlock();
				flashWriteData(BOOT_KEY_ADR,BOOT_KEY);
				flashLock();
                NVIC_SystemReset();
                break;
            default:
                break;
            }
        }
        else if((reciveData->messageId == SET_VECTOR_ID && motionMode==PDU_CONTROL) || (reciveData->messageId==ROS_CONTROL_ID && motionMode==ROS_CONTROL))
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
