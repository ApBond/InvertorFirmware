#include "PWMControl.h"
#include <math.h>

void PWMInit(void)
{
    GPIOA->MODER |= GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
    GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
    GPIOA->AFR[0] |= 6<<GPIO_AFRL_AFRL7_Pos;
    GPIOA->AFR[1] |= 6<<GPIO_AFRH_AFRH0_Pos | 6<<GPIO_AFRH_AFRH1_Pos | 6<<GPIO_AFRH_AFRH2_Pos;
    GPIOB->AFR[0] |= 6<<GPIO_AFRL_AFRL0_Pos | 6<<GPIO_AFRL_AFRL1_Pos;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->CR1 |= TIM_CR1_CMS_0;//Central align mode;
    TIM1->CCMR1 |= 0x06<<TIM_CCMR1_OC1M_Pos | 0x06<<TIM_CCMR1_OC2M_Pos | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 |= 0x06<<TIM_CCMR2_OC3M_Pos | 0x06<<TIM_CCMR2_OC4M_Pos | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
    TIM1->PSC = PWM_PRESCALER-1;
    TIM1->ARR = PWM_ARR;
	
	TIM1->CR2 |= 0b10<<TIM_CR2_MMS2_Pos;//Enable CCR4 trigger

    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR0_1;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR1_1;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR7_1 | GPIO_PUPDR_PUPDR8_1 | GPIO_PUPDR_PUPDR9_1 | GPIO_PUPDR_PUPDR10_1;
}

void PWMStart(void)
{
	
    TIM1->CCR1=0;
	TIM1->CCR2=0;
	TIM1->CCR3=0;
    S_AH;
	S_AL;
	S_BH;
	S_BL;
	S_CH;
	S_CL;
	TIM1->BDTR|=TIM_BDTR_MOE;
    TIM1->CR1|=TIM_CR1_CEN;
    hallInitAngle((GPIOA->IDR&(1<<0)),((GPIOB->IDR&(1<<3))>>3),((GPIOA->IDR&(1<<2))>>2));
	SVPWM_realise_dq(UD,UQ,getAngle(),U_SOURSE);
}

void PWMStop(void)
{
    TIM1->BDTR&=~TIM_BDTR_MOE;
    TIM1->CCR1=0;
	TIM1->CCR2=0;
	TIM1->CCR3=0;
    R_AH;
	R_AL;
	R_BH;
	R_BL;
	R_CH;
	R_CL;
    TIM1->CR1&=~TIM_CR1_CEN;	
}

void SVPWM_realise_dq(float ud,float uq,float tetta,float Ud)
{
    static uint16_t timeA=0,timeB=0,timeC=0;
	float cosElAngle=getCosElAngle();
    float sinElAngle=getSinElAngle();
	float alpha=uq*cosElAngle+ud*sinElAngle;
	float betta=-uq*sinElAngle+ud*cosElAngle;
	uint16_t PWMPeriod=TIM1->ARR;
	float u_alpha=alpha*SQRT_3*PWMPeriod;
	float u_betta=-1*PWMPeriod*betta;
	float X = u_betta;
	float Y = (u_alpha+u_betta)/2;
	float Z=(u_betta-u_alpha)/2;
	if (Y < 0) {
		if (Z < 0) 
		{
			//Sector 5
			timeA=(uint16_t)(PWMPeriod/4 + (PWMPeriod/2 + Y - Z)/2);
			timeB=(uint16_t)(timeA+Z);
			timeC=(uint16_t)(timeA-Y);
		} 
		else /* Z >= 0 */
		{
			if (X <= 0) {
				//Sector 4
				timeA=(uint16_t)(PWMPeriod/4 + (PWMPeriod/2 + X - Z)/2);
				timeB=(uint16_t)(timeA+Z);
				timeC=(uint16_t)(timeB-X);
			} 
			else /* X > 0 */
			{
				//Sector 3
				timeA=(uint16_t)(PWMPeriod/4 + (PWMPeriod/2 + Y - X)/2);
				timeB=(uint16_t)(timeC+X);
				timeC=(uint16_t)(timeA-Y);
			}
		}
	} 
	else /* Y >= 0 */
	{
		if (Z >= 0) {
			//Sector 2
			timeA=(uint16_t)(PWMPeriod/4 + (PWMPeriod/2 + Y - Z)/2);
			timeB=(uint16_t)(timeA+Z);
			timeC=(uint16_t)(timeA-Y);
		} 
		else /* Z < 0 */
		{
			if (X <= 0) {
				//Sector 6
				timeA=(uint16_t)(PWMPeriod/4 + (PWMPeriod/2 + Y - X)/2);
				timeB=(uint16_t)(timeC+X);
				timeC=(uint16_t)(timeA-Y);
			} 
			else /* X > 0 */
			{
				//Sector 1
				timeA=(uint16_t)(PWMPeriod/4 + (PWMPeriod/2 + X - Z)/2);
				timeB=(uint16_t)(timeA+Z);
				timeC=(uint16_t)(timeB-X);
			}
		}
	}
	TIM1->CCR1=timeA;
	TIM1->CCR2=timeB;
	TIM1->CCR3=timeC;
}