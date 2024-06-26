#include "stm32f4xx.h"

void UART_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1; 

    GPIOA->AFR[0] |= 0x00007700;

    USART2->BRR = 0x683;

    USART2->CR1 |= USART_CR1_UE;

    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;

    USART2->CR1 |= USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART2_IRQn);
}

void UART_Transmit(char c) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void UART_Transmit_String(char* str) {
    while (*str) {
        UART_Transmit(*str++);
    }
}

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        char c = USART2->DR;
        UART_Transmit(c);
    }
}

int main(void) {
    UART_Init();
    while (1) {
        UART_Transmit_String("Hello World\n");
        for (int i =0; i< 9000000; i++);
	}
}