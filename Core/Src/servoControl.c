#include "servoControl.h"

int pwm_prev[4]={0};
int pwm_count[4]={0};
int pwm_aim[4]={0};
float dgam[4];
int d_pwm=0;

uint8_t i2c_transmit_buffer[8];
uint8_t i2c_receive_buffer[2];
uint8_t i2c_transmit_counter;

uint8_t calibration(){
	float mesuerment;
	TIM16->DIER&=~TIM_DIER_CC1IE;//Настройка прерываний по совпадению
	mesuerment = i2c_receive_buffer[0] + i2c_receive_buffer[1]<<8;
	while (mesuerment<2000 || mesuerment>2080){
		getServoAngle();
		while(!(DMA1->ISR & DMA_ISR_TCIF7));
		DMA1->IFCR = DMA_IFCR_CTCIF7;
		TIM16->CCR1 = 50;
		if (mesuerment<2000) {//ch1
			GPIOB->BRR=1 << GPIO_BSRR_BR_5;
		}
		else if (mesuerment>2080) {
			GPIOB->BSRR=1 << GPIO_BSRR_BR_5;
		}
	}
	TIM16->CCR1 = 0xffff;
	TIM16->DIER|=TIM_DIER_CC1IE;//Настройка прерываний по совпадению
}

void getServoAngle()
{
	uint8_t addr=0b10010000;
	I2C_Receiver(addr,  2);
}

void setServoAngle(float* targetAngle){
    float gm;
	uint8_t i;
	int current_time;
	if (pwm_count[WHEEL] != pwm_aim[WHEEL]){
		for (i = 0; i < 4; i++) pwm_count[i]=pwm_prev[i] + fabs((pwm_aim[WHEEL] - pwm_count[WHEEL]))/dgam[WHEEL]*(pwm_aim[i]-pwm_count[i]);
	}
	else		
		for (i = 0; i < 4; i++) pwm_count[i]=pwm_aim[i];

	pwm_aim[FL]=targetAngle[FL]*deg*RESOLUTION*REDUCTOR/360;
	pwm_aim[FR]=targetAngle[FR]*deg*RESOLUTION*REDUCTOR/360;
	pwm_aim[RL]=targetAngle[RL]*deg*RESOLUTION*REDUCTOR/360;
	pwm_aim[RR]=targetAngle[RR]*deg*RESOLUTION*REDUCTOR/360;
	
	dgam[FL]=fabs(pwm_aim[FL]-pwm_count[FL]); 
	dgam[FR]=fabs(pwm_aim[FR]-pwm_count[FR]);
	dgam[RL]=fabs(pwm_aim[RL]-pwm_count[RL]);
	dgam[RR]=fabs(pwm_aim[RR]-pwm_count[RR]);	
	for (i=0; i<4; i++) pwm_prev[i]=pwm_count[i];

	gm=dgam[FL];
	if (gm<dgam[FR]) gm=dgam[FR];
	if (gm<dgam[RL]) gm=dgam[RL];
	if (gm<dgam[RR]) gm=dgam[RR];
	TIM16->ARR=150*gm/dgam[WHEEL];
	TIM16->PSC=150*gm/dgam[WHEEL]*400;
	TIM16->ARR=dgam[WHEEL];

	if (pwm_aim[WHEEL]>pwm_count[WHEEL]) {//ch1
		GPIOB->BRR=1 << GPIO_BSRR_BR_5;
		TIM16->CCR1 = 50;
		d_pwm=1;
	}
	else if (pwm_aim[WHEEL]<pwm_count[WHEEL]) {
		TIM16->CCR1 = 50;
		d_pwm=-1;
		GPIOB->BSRR=1 << GPIO_BSRR_BR_5;
	}
	else TIM16->CCR1 = 0xffff;
}

void TIM16_IRQHandler(void)
{
	if (TIM16->SR & TIM_SR_CC1IF) {
		if (pwm_count[WHEEL]==pwm_aim[WHEEL]) TIM1->CCR1 = 0xffff;
		else pwm_count[WHEEL]+=d_pwm;
		TIM16->SR&=~TIM_SR_CC1IF;
	}
}

void tim16Init(void)
{
	//PB5-DIR
	//PB4-step

	RCC->APB2ENR|=RCC_AHBENR_GPIOBEN;
	GPIOB->MODER|=GPIO_MODER_MODER5_0;//dir
	//GPIOB->OTYPER|=GPIO_OTYPER_OT_5;
	
	GPIOB->MODER|=GPIO_MODER_MODER4_1;//step
	//GPIOB->OTYPER|=GPIO_OTYPER_OT_4;
	GPIOB->AFR[0]=(1<<GPIO_AFRL_AFRL4_Pos);//AF1 enable

	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	TIM16->CCER = 0;
	TIM16->ARR = 150;
	TIM16->PSC = 400-1;
	TIM16->BDTR |= TIM_BDTR_MOE;
	TIM16->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM16->CCER |= TIM_CCER_CC1E;
	TIM16->CR1 |= TIM_CR1_CEN;
	TIM16->DIER|=TIM_DIER_CC1IE;//Настройка прерываний по совпадению
	TIM16->CCR1=0xffff;
	NVIC_EnableIRQ(TIM16_IRQn);
}

void I2CInit(){	
	uint8_t conf[3]={0b00000001, 0b10000100, 0b10000011};
	RCC->APB2ENR|=RCC_AHBENR_GPIOBEN;
	GPIOB->MODER|=GPIO_MODER_MODER6_1;//scl
	GPIOB->AFR[1]=(4<<24);//AF4 enable	
	GPIOB->MODER|=GPIO_MODER_MODER7_1;//sda
	GPIOB->AFR[1]=(4<<28);//AF4 enable

	I2C1->TIMINGR=0xB<<I2C_TIMINGR_PRESC_Pos;
	I2C1->TIMINGR=0x13<<I2C_TIMINGR_SCLL_Pos;
	I2C1->TIMINGR=0xF<<I2C_TIMINGR_SCLH_Pos;
	I2C1->TIMINGR=0x2<<I2C_TIMINGR_SDADEL_Pos;
	I2C1->TIMINGR=0x4<<I2C_TIMINGR_SCLDEL_Pos;

	I2C1->CR1=I2C_CR1_RXDMAEN| I2C_CR1_TXIE | I2C_CR1_PE;
	NVIC_EnableIRQ(I2C1_EV_IRQn); 

	I2C_Transfer(0b1001000, 3, conf);
}

void DMAInit(void){
	RCC->AHBENR|=RCC_AHBENR_DMA1EN;
	DMA1_Channel7->CMAR=(uint32_t)i2c_receive_buffer;
	DMA1_Channel7->CPAR=(uint32_t)&(I2C1->RXDR);
	DMA1_Channel7->CNDTR=2;
	DMA1_Channel7->CCR=DMA_CCR_EN | DMA_CCR_MINC;//| DMA_CCR_TCIE
	//NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

void I2C_Transfer(uint8_t addr, uint8_t num_bytes, uint8_t *data){
	uint8_t i;
	I2C1->CR2=addr<<1;
	I2C1->CR2|=1<<I2C_CR2_RD_WRN_Pos;
	I2C1->CR2|=I2C_CR2_START;
	I2C1->CR2|=num_bytes<<I2C_CR2_NBYTES_Pos;

	for (i = 0; i < num_bytes; i++) {
		i2c_transmit_buffer[i]=data[i];
	}
	i2c_transmit_counter=0;
	I2C1->TXDR=i2c_transmit_buffer[i2c_transmit_counter];
}

void I2C_Receiver(uint8_t addr, uint8_t num_bytes){
	I2C1->CR2=addr<<1;
	I2C1->CR2&=~(1<<I2C_CR2_RD_WRN_Pos);
	I2C1->CR2|=I2C_CR2_START;
	I2C1->CR2|=num_bytes<<I2C_CR2_NBYTES_Pos;
}

void I2C1_EV_IRQHandler(void) {
	if(I2C1->ISR & I2C_ISR_TXIS){
		i2c_transmit_counter++;
		I2C1->TXDR=i2c_transmit_buffer[i2c_transmit_counter];
	}
}