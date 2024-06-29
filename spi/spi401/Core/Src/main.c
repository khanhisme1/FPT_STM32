#include "stm32f4xx.h"

void SPI1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
    GPIOA->MODER |= GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;
    GPIOA->AFR[0] |= (5 << 20) | (5 << 24) | (5 << 28);

    SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SPE;
}

void SPI2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	
    GPIOB->MODER |= GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1 | GPIO_MODER_MODE15_1;
    GPIOB->AFR[1] |= (5 << 20) | (5 << 24) | (5 << 28);

    SPI2->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1;
    SPI2->CR1 &= ~SPI_CR1_MSTR;
    SPI2->CR1 &= ~(SPI_CR1_SSM | SPI_CR1_SSI);
    SPI2->CR1 |= SPI_CR1_SPE;
}

void SPI_Transmit(SPI_TypeDef *SPIx, uint8_t data) {
    while (!(SPIx->SR & SPI_SR_TXE));
    SPIx->DR = data;
    while (SPIx->SR & SPI_SR_BSY);
}

uint8_t SPI_Receive(SPI_TypeDef *SPIx) {
    while (!(SPIx->SR & SPI_SR_RXNE));
    return SPIx->DR;
}

void UART2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    GPIOA->MODER |= GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1;
    GPIOA->AFR[0] |= 0x7700;
    
    USART2->BRR = 0x683;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void UART2_Send(uint8_t data) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = data;
}

int main(void) {
    SPI1_Init();
    SPI2_Init();
		UART2_Init();
    uint8_t tx_data = 'h';
    uint8_t rx_data;

    while (1) {
				SPI_Transmit(SPI1, tx_data);
				rx_data = SPI_Receive(SPI2);
				UART2_Send(rx_data);
				for (int i = 0 ; i < 10000000; i++);
    }
}
