#include "currentSensing.h"
#include <math.h>

static abc_t Iabc;
static struct d_q_t Idq;
static ADC_state_t ADCstate;
static uint32_t phaseAOffsetFactor;
static uint32_t phaseBOffsetFactor;
static uint32_t phaseCOffsetFactor;

static void ADCGpioInit(void)
{
    GPIOA->MODER |= GPIO_MODER_MODER1 | GPIO_MODER_MODER5 | GPIO_MODER_MODER5;
}

void ADC1_2_IRQHandler(void)
{
    static uint16_t i;
    static uint16_t errorCounter;
    int16_t currentA=0;
    int16_t currentC=0;
    int16_t currentB=0;
    alpha_betta_t I_alpha_betta;
    float rotorElAngle;
    float cosElAngle;
    float sinElAngle;
    static struct d_q_t IdqBuff[CURRENT_MEASURE_COUNT];
    if(ADC1->ISR & ADC_ISR_JEOC)
    {
        ADC1->ISR|=ADC_ISR_JEOC;
        if(ADCstate==ADC_CALIBRATION)
        {
            phaseAOffsetFactor+=ADC1->JDR1;
            phaseCOffsetFactor+=ADC2->JDR1;
            i++;
            if(i==ADC_CALIBRATION_COUNT)
            {
                phaseAOffsetFactor/=ADC_CALIBRATION_COUNT;
                phaseCOffsetFactor/=ADC_CALIBRATION_COUNT;
                ADCstate=ADC_OK;
                i=0;
            }
            else
                ADC1->CR |= ADC_CR_JADSTART;
        }
        else
        {
            currentA=ADC1->JDR1-phaseAOffsetFactor;
            currentC=ADC2->JDR1-phaseCOffsetFactor;
            Iabc.a=ADC_TO_AMP*currentA;
            Iabc.c=ADC_TO_AMP*currentC;
            Iabc.b=-Iabc.a-Iabc.c;
            I_alpha_betta.alpha=Iabc.a;
            I_alpha_betta.betta=-(Iabc.a+2*Iabc.b)/SQRT_3;
            rotorElAngle=getAngle();
            cosElAngle=getCosElAngle();
            sinElAngle=getSinElAngle();
            Idq.d=I_alpha_betta.alpha*sinElAngle+I_alpha_betta.betta*cosElAngle;
            Idq.q=I_alpha_betta.alpha*cosElAngle-I_alpha_betta.betta*sinElAngle;
            //Idq.d=Ia;
            //Idq.q=-Ia-Ic;
            if(fabs(Iabc.a)>=CURRENT_LIM || fabs(Iabc.b)>=CURRENT_LIM || fabs(Iabc.c)>=CURRENT_LIM)
            {
                if(errorCounter==10)
                {
                    setErrorState(OVERCURRENT_ERROR);
                    errorCounter=0;
                    sendErrorState();
                }
                errorCounter++;
            }
            else
            {
                errorCounter=0;
            }
            currentLoop(Idq.d,Idq.q,rotorElAngle);
        }
    }
}

ADC_state_t ADCInit(void)
{
    uint32_t time;
    //ADC1 Init process
    ADCGpioInit();
    RCC->AHBENR |= RCC_AHBENR_ADC12EN;
    RCC->CFGR2 |= 0b10000 << RCC_CFGR2_ADCPRE12_Pos;//SET ADC clock source to PLL
    ADC1->CR=0;
    ADC1->CR |= ADC_CR_ADVREGEN_0;//Voltage regulator enable
    delay_ms(1);//Wait for voltage regulator started
    ADC1->CR |= ADC_CR_ADCAL;//Start ADC calibration
    time=0;
    while(ADC1->CR & ADC_CR_ADCAL)//Wait for ADC calibration ended
    {
        if(time==0xFFFFFFFF) return ADC_NOT_INIT;
        time++;
    }
    //ADC2 Init process
    ADC2->CR=0;
    ADC2->CR |= ADC_CR_ADVREGEN_0;//Voltage regulator enable
    delay_ms(1);//Wait for voltage regulator started
    ADC2->CR |= ADC_CR_ADCAL;//Start ADC calibration
    time=0;
    while(ADC2->CR & ADC_CR_ADCAL)//Wait for ADC calibration ended
    {
        if(time==0xFFFFFFFF) return ADC_NOT_INIT;
        time++;
    }
    ADC12_COMMON->CCR |= 0b101;

    ADC1->CR |= ADC_CR_ADEN;// ADC1 enable
    ADC2->CR |= ADC_CR_ADEN;// ADC2 enabl
    ADC1->SMPR1 |= 0b100 << ADC_SMPR1_SMP2_Pos;
    ADC2->SMPR1 |= 0b100 << ADC_SMPR1_SMP2_Pos;
    
    //Channal init
    ADC1->JSQR |= 2 << ADC_JSQR_JSQ1_Pos;
    ADC2->JSQR |= 2 << ADC_JSQR_JSQ1_Pos;
    ADC1->IER |= ADC_IER_JEOCIE;// Enable ADC1(master) injected convertion interrupt 
    NVIC_EnableIRQ(ADC1_2_IRQn); 
    NVIC_SetPriority(ADC1_2_IRQn,0);
    delay_ms(10);//Wait for ADC enable
    ADCstate=ADC_CALIBRATION;//Start measuring offset voltage
    ADC1->CR |= ADC_CR_JADSTART;
    while(ADCstate==ADC_CALIBRATION)
    {
        delay_ms(1);
    }
    ADC1->JSQR |= 8<<ADC_JSQR_JEXTSEL_Pos | ADC_JSQR_JEXTEN_0 ;//Enable TIM1 CCR4 trigger 
    ADC1->CR |= ADC_CR_JADSTART;
    return ADC_OK;
}

abc_t getCurrentIabc(void)
{
    return Iabc;
}

struct d_q_t getCurrentIdq(void)
{
    return Idq;
}