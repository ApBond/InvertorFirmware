#include "hallFeedback.h"

static float elAngle=0;
static float mSpeed;

static float hallGetAngle(uint8_t ha,uint8_t hb,uint8_t hc)
{
	static uint8_t prevHallState=0;
	float angle=0;
	uint8_t hallState=hc<<2 | hb<<1 | ha;
	switch(hallState)
	{
		case 5:
			if(prevHallState==4)
			{
				angle=OFFSET_ANGLE_RAD;
			}
			else if(prevHallState==1)
			{
				angle=OFFSET_ANGLE_RAD+DEG_60;
			}
			break;
		case 1:
			if(prevHallState==5)
			{
				angle=OFFSET_ANGLE_RAD+DEG_60;
			}
			else if(prevHallState==3)
			{
				angle=OFFSET_ANGLE_RAD+2*DEG_60;
			}
			break;
		case 3:
			if(prevHallState==1)
			{
				angle=OFFSET_ANGLE_RAD+2*DEG_60;
			}
			else if(prevHallState==2)
			{
				angle=OFFSET_ANGLE_RAD+3*DEG_60;
			}
			break;
		case 2:
			if(prevHallState==3)
			{
				angle=OFFSET_ANGLE_RAD+3*DEG_60;
			}
			else if(prevHallState==6)
			{
				angle=OFFSET_ANGLE_RAD-2*DEG_60;
			}
			break;
		case 6:
			if(prevHallState==2)
			{
				angle=OFFSET_ANGLE_RAD-2*DEG_60;
			}
			else if(prevHallState==4)
			{
				angle=OFFSET_ANGLE_RAD-DEG_60;
			}
			break;
		case 4:
			if(prevHallState==6)
			{
				angle=OFFSET_ANGLE_RAD-DEG_60;
			}
			else if(prevHallState==5)
			{
				angle=OFFSET_ANGLE_RAD;
			}
			break;
	}
	prevHallState=hallState;
	return angle;
}

void TIM2_IRQHandler(void)
{
    static uint8_t state=0;
    static uint64_t speedBuff=0;
    uint32_t elPeriod;
    float mPeriod;
	if(TIM2->SR & TIM_SR_CC1IF)
	{
        TIM2->SR&=~TIM_SR_CC1IF;
		elAngle=hallGetAngle((GPIOA->IDR&(1<<0)),((GPIOB->IDR&(1<<3))>>3),((GPIOA->IDR&(1<<2))>>2));
		//SVPWM_realise_dq(UD,UQ,elAngle,U_SOURSE);
		if(state<HALL_MEASURE_COUNT-1)
		{
			speedBuff+=TIM2->CCR1;
			state++;
		}
		else
		{
			speedBuff+=TIM2->CCR1;
			elPeriod=speedBuff/HALL_MEASURE_COUNT;
				
			
			//sixStep(GPIOA->IDR&(1<<0),((GPIOA->IDR&(1<<1))>>1),((GPIOB->IDR&(1<<10))>>10));

			//TIM2->SR&=~TIM_SR_CC1IF;
			//angleInterpolPeriod=(uint32_t)((float)TIM2->CCR1*6*d_ANGLE_RAD/(2*PI));
			mPeriod=elPeriod*6/POLE_PAIRS;
			mSpeed=60*72000000/mPeriod;
			/*angleInterpolPeriod=(uint32_t)((float)elPeriod*d_ANGLE_RAD/PI_3);
			TIM5->CR1&=~TIM_CR1_CEN; 
			TIM5->CNT=0;
			TIM5->ARR=angleInterpolPeriod;
			period=angleInterpolPeriod-1;
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
			TIM5->CR1|=TIM_CR1_CEN;*/
			state=0;
			speedBuff=0;
		}
	}
}

void TIM3_IRQHandler(void)
{
    static uint8_t state=0;
    if(TIM3->SR & TIM_SR_UIF)
    {
        TIM3->SR&=~TIM_SR_UIF;
        if(state)
        {
            GPIOB->BSRR|=GPIO_BSRR_BR_5;
            state=0;
        }
        else
        {
            GPIOB->BSRR|=GPIO_BSRR_BS_5;
            state=1;
        }
    }
}

void hallTimInit(void)
{
    GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER2_1;
    GPIOB->MODER |= GPIO_MODER_MODER3_1;
    GPIOA->AFR[0] |= 1<<GPIO_AFRL_AFRL0_Pos | 1<<GPIO_AFRL_AFRL2_Pos;
    GPIOB->AFR[0] |= 1<<GPIO_AFRL_AFRL3_Pos;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CR2 |= TIM_CR2_TI1S;
    TIM2->CR2 |= 0x05<<TIM_CR2_MMS_Pos;
    TIM2->SMCR |= 0x04<<TIM_SMCR_TS_Pos | 0x04<<TIM_SMCR_SMS_Pos;
    TIM2->CCMR1 |= 0x03 << TIM_CCMR1_CC1S_Pos | HALL_FILTER<<TIM_CCMR1_IC1F_Pos | HALL_FILTER<<TIM_CCMR1_IC2F_Pos;
    TIM2->CCMR2 |= HALL_FILTER<<TIM_CCMR2_IC3F_Pos;
    TIM2->CCER |= TIM_CCER_CC1E;
    TIM2->DIER |= TIM_DIER_CC1IE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;

    /*RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->ARR = 7200-1;
    TIM3->CR1|=TIM_CR1_CEN;

    GPIOB->MODER|=GPIO_MODER_MODER5_0;*/
}

float getSpeed(void)
{
    return mSpeed;
}
