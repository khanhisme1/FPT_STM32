#include "stm32f10x.h"

#define ADC1_CR2_ADON (1 << 0) 
#define ADC1_CR2_CONT (1 << 1) 
#define ADC1_SMPR2_SMP1 (7 << 3) 
#define ADC1_SQR3_SQ1 (1 << 0) 
#define USART1_CR1_UE (1 << 13) 
#define USART1_CR1_TE (1 << 3) 
#define USART1_CR1_RE (1 << 2) 
#define USART1_BRR_9600 ((uint16_t)0x341) 
#define ADC1_DR_Address ((uint32_t)0x4001244C)

volatile uint16_t ADCConvertedValue;

void DMA_Config(void) {
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel1->CCR = 0;
	DMA1_Channel1->CNDTR = 0;
	DMA1_Channel1->CPAR = 0;
	DMA1_Channel1->CMAR = 0;
	DMA1_Channel1->CPAR = ADC1_DR_Address;
	DMA1_Channel1->CMAR = (uint32_t)&ADCConvertedValue;
	DMA1_Channel1->CNDTR = 1;
	DMA1_Channel1->CCR |= DMA_CCR1_MINC | DMA_CCR1_PSIZE_0 | DMA_CCR1_MSIZE_0 | DMA_CCR1_TEIE | DMA_CCR1_EN; 
}

void ADC_Config(void) {
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;
	GPIOA->CRL &= ~GPIO_CRL_CNF1;
	GPIOA->CRL &= ~GPIO_CRL_MODE1;
	ADC1->CR2 |= ADC1_CR2_ADON | ADC1_CR2_CONT;
	ADC1->SMPR2 |= ADC1_SMPR2_SMP1;
	ADC1->SQR3 |= ADC1_SQR3_SQ1;
	ADC1->CR2 |= ADC_CR2_DMA; 
}

void USART_Config(void) {
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;
	GPIOA->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1;
	GPIOA->CRH &= ~GPIO_CRH_MODE10;
	GPIOA->CRH &= ~GPIO_CRH_CNF10_0;
	USART1->CR1 |= USART1_CR1_UE | USART1_CR1_TE | USART1_CR1_RE;
	USART1->BRR = USART1_BRR_9600; 
}

void USART_SendData(uint16_t Data) {
	ADC1->CR2 |= ADC1_CR2_ADON;
	while(!(ADC1->SR & ADC_SR_EOC));
	while(!(USART1->SR & USART_SR_TXE));
	USART1->DR = (Data >> 4) & (uint16_t)0xFFFF; 
}

void TIM_Config(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	GPIOA->CRL &= ~GPIO_CRL_CNF0;
	GPIOA->CRL |= GPIO_CRL_CNF0_1;
	GPIOA->CRL |= GPIO_CRL_MODE0;

	TIM2->PSC = 7199;
	TIM2->ARR = 9999;
	TIM2->CCR1 = 4999;
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM2->CCER |= TIM_CCER_CC1E;

	TIM2->CR1 |= TIM_CR1_CEN;

}

int main(void) {
	TIM_Config();
	DMA_Config();
	ADC_Config();
	USART_Config();
	while(1) {
		USART_SendData(ADC1->DR);
	}
}



