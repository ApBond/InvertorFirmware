#include "control.h"
    
d_q_t Us;

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

extern float Ia;
extern float Ic;
extern float Ib;

d_q_t circleLimitation(d_q_t vector)
{
    float U = vector.d*vector.d+vector.q*vector.q;
    float sqrtU;
    d_q_t rez;
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
    d_q_t Is;
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

// d_q_t circleLimitation(d_q_t vector)
// {

// }

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
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    TIM16->PSC = 7200-1;
    TIM16->ARR = SPEED_LOOP_TIM_ARR-1;
    TIM16->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM16_IRQn);
}

void startSpeedLoop(void)
{
    TIM16->CR1 |= TIM_CR1_CEN;
}

void stopSpeedLoop(void)
{
    TIM16->CR1&=~TIM_CR1_CEN;
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

void TIM16_IRQHandler(void)
{
    if(TIM16->SR & TIM_SR_UIF)
    {
        TIM16->SR &=~TIM_SR_UIF;
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
