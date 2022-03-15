#include "control.h"
    
static struct d_q_t Us;
static FOC_Error_t errorState = NOT_ERROR;
static FOC_Mode_t controleMode = SPEED_CONTROL;
static float referenceSpeed=40;
static float referenceTorque=0;

PIDHandle_t idLoopPID=
{
    .kp=0.004,
    .ki=0.000008,
    .kd=0,
    .prevError=0,
    .integralTerm=0
};

PIDHandle_t iqLoopPID=
{
    .kp=0.004,
    .ki=0.000008,
    . kd=0,
    .prevError=0,
    .integralTerm=0
};

PIDHandle_t speedLoopPID=
{
    .kp=0.1,
    .ki=0.00004,
    . kd=0,
    .prevError=0,
    .integralTerm=0
};

struct d_q_t circleLimitation(struct d_q_t vector)
{
    float U = vector.d*vector.d+vector.q*vector.q;
    float sqrtU;
    struct d_q_t rez;
    if(U>1)
    {
        sqrtU=sqrt(U);
        rez.d=vector.d/sqrtU;
        rez.q=vector.q/sqrtU;
        return rez;
    }
    return vector;
}

void currentLoop(float Id,float Iq,float rotorElAngle)
{
    struct d_q_t Is;
    alpha_betta_t I_alpha_betta;
    Us.d = PIDController(&idLoopPID,0-Id);
    Us.q = PIDController(&iqLoopPID,referenceTorque-Iq);
    Us=circleLimitation(Us);
    SVPWM_realise_dq(Us.d,Us.q,rotorElAngle,12);
}

void speedLoop(void)
{
    float torque;
    referenceTorque=PIDController(&speedLoopPID,referenceSpeed-getSpeed());
    //referenceTorque=2;
    //setReferenceTorque(torque);   
}

void regulatorClear(void)
{
    idLoopPID.integralTerm=0;
    idLoopPID.prevError=0;
    iqLoopPID.prevError=0;
    iqLoopPID.integralTerm=0;
    speedLoopPID.prevError=0;
    speedLoopPID.integralTerm=0;
}

void speedLoopTimerInit(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    TIM15->PSC = 7200-1;
    TIM15->ARR = SPEED_LOOP_TIM_ARR-1;
    TIM15->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM15_IRQn);
}

void startSpeedLoop(void)
{
    TIM15->CR1 |= TIM_CR1_CEN;
}

void stopSpeedLoop(void)
{
    TIM15->CR1&=~TIM_CR1_CEN;
}

void setReferenceSpeed(float speed)
{
    if(speed>=MAX_SPEED_RPM)
        referenceSpeed=MAX_SPEED_RPM;
    else if (-1*MAX_SPEED_RPM<=speed)
        referenceSpeed=-1*MAX_SPEED_RPM;
    else
        referenceSpeed=speed;
}

void setReferenceTorque(float torque)
{
    if(torque>=MAX_TORQUE_AMP)
        referenceTorque=MAX_TORQUE_AMP;
    else if (-1*MAX_TORQUE_AMP<=torque)
        referenceTorque=-1*MAX_TORQUE_AMP;
    else
        referenceTorque=torque;
}

float getReferenceTorque(void)
{
    return referenceTorque;
}

float getReferenceSpeed(void)
{
    return referenceSpeed;
}

struct d_q_t getControllImpact(void)
{
    return Us;
}

void TIM15_IRQHandler(void)
{
    if(TIM15->SR & TIM_SR_UIF)
    {
        TIM15->SR &=~TIM_SR_UIF;
        speedLoop();
    }
}

float PIDController(PIDHandle_t * PID,float error)
{
    float deltaError = error - PID->prevError;
    float controllerOut;
    PID->prevError=error;
    PID->integralTerm+=error;
    controllerOut=error*PID->kp+PID->integralTerm*PID->ki+deltaError*PID->kd;
    return controllerOut;
}

void motorStart(void)
{
    if(errorState==NOT_ERROR)
    {
        if(controleMode==SPEED_CONTROL)
        {
            regulatorClear();
            PWMStart();
            startSpeedLoop();
        }
        else if (controleMode==TORQUE_CONTROL)
        {
            regulatorClear();
            PWMStart();
        }
    }
}

void motorStop(void)
{
    PWMStop();  
    stopSpeedLoop(); 
    regulatorClear();
}

void setControlMode(FOC_Mode_t mode)
{
    controleMode=mode;
}

void setPIDSettings(Controller_type_t type,PIDHandle_t controller)
{
    switch(type)
    {
        case SPEED_PID:
            speedLoopPID=controller;
            break;
        case ID_PID:
            idLoopPID=controller;
            break;
        case IQ_PID:
            iqLoopPID=controller;
            break;
    }
}

void setErrorState(FOC_Error_t error)
{
    errorState=NOT_ERROR;
    if(error!=NOT_ERROR)
    {
        motorStop();
    }
}