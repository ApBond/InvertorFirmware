#include "servoControl.h"

#define ZERO_POINT 803
#define DEAD_ZONE 2

int pwm_prev[4]={0};
int pwm_count[4]={0};
int pwm_aim[4]={0};
int dgam[4];
int d_pwm=0;
static uint8_t reciveFlag=0;

uint8_t i2c_transmit_buffer[8];
uint8_t i2c_receive_buffer[2];
uint8_t i2c_transmit_counter;
static uint8_t reciverState=0;
uint16_t mesuerment;

uint8_t calibration()
{
	TIM16->CR1&=~TIM_CR1_CEN;
	TIM16->DIER&=~TIM_DIER_CC1IE;
	TIM16->ARR = 150;
	TIM16->PSC = 2000-1;
	TIM16->CR1 |= TIM_CR1_CEN;
	int error;
	while(1)
	{	
		if(reciveFlag==0) getServoAngle();
		if(reciveFlag==2)
		{
			mesuerment = ((i2c_receive_buffer[0]<<8)+i2c_receive_buffer[1])>>4;
			error=mesuerment-ZERO_POINT;
			if(abs(error)>DEAD_ZONE)
			{
				//TIM16->CCR1 = 50;
				#ifdef FRONT_LEFT 
				if(error<0)
				{
					GPIOB->BSRR= GPIO_BSRR_BS_5;
				}
				else if(error>0)
				{
					GPIOB->BSRR= GPIO_BSRR_BR_5;
				}
				#endif
				#ifdef REAR_LEFT | REAR_RIGHT 
				if(error>0)
				{
					GPIOB->BSRR= GPIO_BSRR_BS_5;
				}
				else if(error<0)
				{
					GPIOB->BSRR= GPIO_BSRR_BR_5;
				}
				#endif
				#ifdef REAR_RIGHT 
				if(error>0)
				{
					GPIOB->BSRR= GPIO_BSRR_BS_5;
				}
				else if(error<0)
				{
					GPIOB->BSRR= GPIO_BSRR_BR_5;
				}
				#endif
			}
			else
			{
				TIM16->CCR1 = 0xffff;
				/*TIM16->CR1&=~TIM_CR1_CEN;
				TIM16->PSC = 50-1;
				TIM16->DIER|=TIM_DIER_CC1IE;
				TIM16->CR1 |= TIM_CR1_CEN;
				return;*/
			}
			reciveFlag=0;
			delay_ms(1);
		}		
	}
	
	/*while (mesuerment<2000 || mesuerment>2080){
		getServoAngle();
		while(!(DMA1->ISR & DMA_ISR_TCIF7));
		TIM16->CCR1 = 50;
		if (mesuerment<2000) {//ch1
			GPIOB->BRR= GPIO_BSRR_BS_5;
		}
		else if (mesuerment>2080) {
			GPIOB->BSRR= GPIO_BSRR_BR_5;
		}
	}
	TIM16->CCR1 = 0xffff;
	TIM16->DIER|=TIM_DIER_CC1IE;//Настройка прерываний по совпадению*/
	
}

void getServoAngle()
{
	I2C_Receiver(0x48, 2);
}

float odomServoGetAngle(void)
{
	float temp;
	temp=(float)(pwm_count[WHEEL]/(deg*RESOLUTION*REDUCTOR));
	temp*=360;
	return (float)temp;
}

void setServoAngle(float* targetAngle){
    float gm;
	uint8_t i;
	int current_time;
	if (pwm_count[WHEEL] != pwm_aim[WHEEL]){
		for (i = 0; i < 4; i++) pwm_count[i]=pwm_prev[i] + (int)((pwm_aim[i]-pwm_prev[i])*fabs((pwm_count[WHEEL]-pwm_prev[WHEEL]))/dgam[WHEEL]);
	}
	else		
		for (i = 0; i < 4; i++) pwm_count[i]=pwm_aim[i];

	pwm_aim[FL]=targetAngle[FL]*deg*RESOLUTION*REDUCTOR/360;
	pwm_aim[FR]=targetAngle[FR]*deg*RESOLUTION*REDUCTOR/360;
	pwm_aim[RL]=targetAngle[RL]*deg*RESOLUTION*REDUCTOR/360;
	pwm_aim[RR]=targetAngle[RR]*deg*RESOLUTION*REDUCTOR/360;
	
	dgam[FL]=(int)(fabs(pwm_aim[FL]-pwm_count[FL])); 
	dgam[FR]=(int)fabs(pwm_aim[FR]-pwm_count[FR]);
	dgam[RL]=(int)fabs(pwm_aim[RL]-pwm_count[RL]);
	dgam[RR]=(int)fabs(pwm_aim[RR]-pwm_count[RR]);	
	for (i=0; i<4; i++) pwm_prev[i]=pwm_count[i];

	gm=dgam[FL];
	if (gm<dgam[FR]) gm=dgam[FR];
	if (gm<dgam[RL]) gm=dgam[RL];
	if (gm<dgam[RR]) gm=dgam[RR];
	TIM16->ARR=1500*gm/dgam[WHEEL];
	//TIM16->PSC=150*gm/dgam[WHEEL]*400;
	//TIM16->ARR=dgam[WHEEL];

	if (pwm_aim[WHEEL]>pwm_count[WHEEL]) {//ch1
		GPIOB->BSRR= GPIO_BSRR_BS_5;
		TIM16->CCR1 = 400;
		d_pwm=1;
	}
	else if (pwm_aim[WHEEL]<pwm_count[WHEEL]) {
		TIM16->CCR1 = 400;
		d_pwm=-1;
		GPIOB->BSRR=GPIO_BSRR_BR_5;
	}
	else TIM16->CCR1 = 0xffff;
}

void TIM16_IRQHandler(void)
{
	if (TIM16->SR & TIM_SR_CC1IF) {
		TIM16->SR&=~TIM_SR_CC1IF;
		if (pwm_count[WHEEL]==pwm_aim[WHEEL]) TIM16->CCR1 = 0xffff;
		else pwm_count[WHEEL]+=d_pwm;
		
	}
}

void tim16Init(void)
{
	//PB5-DIR
	//PB4-step

	RCC->APB2ENR|=RCC_AHBENR_GPIOBEN;
	GPIOB->MODER|=GPIO_MODER_MODER5_0;//dir
	GPIOB->OTYPER|=GPIO_OTYPER_OT_5;
	
	GPIOB->MODER|=GPIO_MODER_MODER4_1;//step
	GPIOB->OTYPER|=GPIO_OTYPER_OT_4;
	GPIOB->AFR[0]|=(1<<GPIO_AFRL_AFRL4_Pos);//AF1 enable
	GPIOB->PUPDR &=~GPIO_PUPDR_PUPDR4;
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	TIM16->CCER = 0;
	TIM16->ARR = 150;
	TIM16->PSC = 50-1;
	TIM16->BDTR |= TIM_BDTR_MOE;
	TIM16->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM16->CCER |= TIM_CCER_CC1E;
	TIM16->CR1 |= TIM_CR1_CEN;
	TIM16->DIER|=TIM_DIER_CC1IE;//Настройка прерываний по совпадению
	TIM16->CCR1=0xffff;
	NVIC_EnableIRQ(TIM16_IRQn);
	NVIC_SetPriority(TIM16_IRQn,15);
}

void I2CInit(){	
	uint8_t conf[3]={0x1, 0b11000000, 0b10000011};
	GPIOB->MODER|=GPIO_MODER_MODER6_1;//scl
	GPIOB->AFR[0]|=(4<<GPIO_AFRL_AFRL6_Pos);//AF4 enable	
	GPIOB->MODER|=GPIO_MODER_MODER7_1;//sda
	GPIOB->AFR[0]|=(4<<GPIO_AFRL_AFRL7_Pos);//AF4 enable
	//GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0;

	RCC->CFGR3|=RCC_CFGR3_I2C1SW;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	I2C1->CR1&=~I2C_CR1_PE;
	delay_ms(1);
	I2C1->CR1|=I2C_CR1_ANFOFF;
	I2C1->TIMINGR=0x10C091CF;
	I2C1->CR1|=I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_PE;
	delay_ms(1);
	NVIC_EnableIRQ(I2C1_EV_IRQn); 

	I2C_Transfer(0x48, 3, conf);
	delay_ms(10);
	I2C_Transfer(0x48, 1, 0);
}

void DMAInit(void){
	RCC->AHBENR|=RCC_AHBENR_DMA1EN;
	DMA1_Channel7->CMAR=(uint32_t)i2c_receive_buffer;
	DMA1_Channel7->CPAR=(uint32_t)&(I2C1->RXDR);
	DMA1_Channel7->CNDTR=2;
	DMA1_Channel7->CCR=DMA_CCR_EN | DMA_CCR_MINC;//| DMA_CCR_TCIE
	//NVIC_EnableIRQ(DMA1_Channe l7_IRQn);
}

void I2C_Transfer(uint8_t addr, uint8_t num_bytes, uint8_t *data){
	uint8_t i;
	for (i = 0; i < num_bytes; i++) {
		i2c_transmit_buffer[i]=data[i];
	}
	i2c_transmit_counter=0;
	I2C1->CR2|=addr<<1 | num_bytes<<16 | I2C_CR2_START;
	//while(!(I2C1->ISR & I2C_ISR_TXE));
	I2C1->TXDR=i2c_transmit_buffer[i2c_transmit_counter];
}

void I2C_Receiver(uint8_t addr, uint8_t num_bytes)
{
	reciverState=0;
	reciveFlag=1;
	I2C1->CR2=addr<<1 | I2C_CR2_RD_WRN | num_bytes<<I2C_CR2_NBYTES_Pos | I2C_CR2_START;
}


void I2C1_EV_IRQHandler(void) {
	if(I2C1->ISR & I2C_ISR_TXIS){
		i2c_transmit_counter++;
		I2C1->TXDR=i2c_transmit_buffer[i2c_transmit_counter];
	}
	if(I2C1->ISR & I2C_ISR_RXNE){
		i2c_receive_buffer[reciverState]=I2C1->RXDR;
		reciverState++;
		if(reciverState==2)
		{
			reciverState=0;
			reciveFlag=2;
		}
	}
}