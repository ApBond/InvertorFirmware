#include "control.h"
    
d_q_t Us;
float referenceSpeed=50;

PIDHandle_t idLoopPID=
{
    .kp=0.0001,
    .ki=0.000003,
    .kd=0,
    .prevError=0,
    .integralTerm=0
};

PIDHandle_t iqLoopPID=
{
    .kp=0.0001,
    .ki=0.000003,
    . kd=0,
    .prevError=0,
    .integralTerm=0
};

PIDHandle_t speedLoopPID=
{
    .kp=0.002,
    .ki=0.000007,
    . kd=0,
    .prevError=0,
    .integralTerm=0
};

extern float Ia;
extern float Ic;
extern float Ib;

void TIM16_IRQHandler(void)
{
    float Uq;
    if(TIM16->SR & TIM_SR_UIF)
    {
        TIM16->SR &=~TIM_SR_UIF;
        Uq=PIDController(&speedLoopPID,referenceSpeed-getSpeed());
        if(Uq>0.5) Uq=0.5;
        if(Uq<-0.5) Uq=-0.5;
        SVPWM_realise_dq(0,Uq,getAngle(),12);
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

void currentLoop(float Id,float Iq,float rotorElAngle)
{
    d_q_t Is;
    alpha_betta_t I_alpha_betta;
    //float rotorElAngle;
    /*I_alpha_betta.alpha=Ia;
    I_alpha_betta.betta=(Ia+2*Ib)/SQRT_3;
    rotorElAngle=getAngle();
    Is.d=I_alpha_betta.alpha*cosf(rotorElAngle)+I_alpha_betta.betta*sinf(rotorElAngle);
    Is.q=I_alpha_betta.betta*cosf(rotorElAngle)-I_alpha_betta.alpha*sinf(rotorElAngle);*/
    //Us.d = PIDController(&idLoopPID,0-Id);
    //Us.d=0;
    //Us.q = PIDController(&iqLoopPID,REF_TORQUE-Iq);
    //if(Us.d>0.7) Us.d=0.7;
    //if(Us.q>0.7) Us.q=0.7;
    //SVPWM_realise_dq(0,0.5,rotorElAngle,12);
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