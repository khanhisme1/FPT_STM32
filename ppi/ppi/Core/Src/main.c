#include "stm32f4xx.h"

void delay(uint16_t ms) {
	for (int i = 0 ; i < 1000 * ms; i++);
}


void GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOA->MODER |= 0x5555;
		GPIOC->MODER |= 0x55;
}

void turn_LED_mode0() {
		GPIOA->ODR = 0x80;
		delay(5000);
		GPIOC->ODR = 0x1;
		delay(5000);
		GPIOC->ODR = 0x3;
		delay(5000);
		GPIOC->ODR = 0x7;
		delay(5000);
		GPIOC->ODR = 0xF;
		delay(5000);
		GPIOC->ODR = 0xE;
		delay(5000);
		GPIOC->ODR = 0xC;
		delay(5000);
		GPIOA->ODR = 0x55;
		delay(5000);
		GPIOC->ODR = 0x7;
		delay(5000);
		GPIOC->ODR = 0xF;
		delay(5000);
		GPIOC->ODR = 0x3;
		delay(5000);
		GPIOC->ODR = 0x1;
		delay(5000);
		GPIOC->ODR = 0x7;
		delay(5000);
		GPIOC->ODR = 0x5;
		delay(5000);
		GPIOC->ODR = 0xB;
		delay(5000);
		GPIOC->ODR = 0x9;
}

int main(void)
{
    GPIO_Init();
		turn_LED_mode0();
    while (1) {
		}
}