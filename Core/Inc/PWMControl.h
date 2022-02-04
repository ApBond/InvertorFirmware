#ifndef __PWM_CONTROL_H
#define __PWM_CONTROL_H

#include "main.h"

#define PWM_PRESCALER 5
#define PWM_ARR 720

#define UD 0
#define UQ 0.5
#define U_SOURSE 12

#define S_AL TIM1->CCER|=TIM_CCER_CC1NE
#define S_AH TIM1->CCER|=TIM_CCER_CC1E
#define S_BL TIM1->CCER|=TIM_CCER_CC2NE
#define S_BH TIM1->CCER|=TIM_CCER_CC2E
#define S_CL TIM1->CCER|=TIM_CCER_CC3NE
#define S_CH TIM1->CCER|=TIM_CCER_CC3E

#define R_AL TIM1->CCER&=~TIM_CCER_CC1NE
#define R_AH TIM1->CCER&=~TIM_CCER_CC1E
#define R_BL TIM1->CCER&=~TIM_CCER_CC2NE
#define R_BH TIM1->CCER&=~TIM_CCER_CC2E
#define R_CL TIM1->CCER&=~TIM_CCER_CC3NE
#define R_CH TIM1->CCER&=~TIM_CCER_CC3E

void PWMInit(void);
void PWMStart(void);
void PWMStop(void);
void SVPWM_realise_dq(float ud,float uq,float tetta,float Ud);

#endif