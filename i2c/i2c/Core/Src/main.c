#include "stm32f1xx.h"

void I2C1_Init(void)
{
		// Enable clock for GPIOB
		// Enable clock for GPIOB and I2C1
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

		// Enable I2C1 event interrupt
		NVIC_EnableIRQ(I2C1_EV_IRQn);

		// Configure PB6 and PB7 as alternate function open-drain
		GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF6 | GPIO_CRL_CNF7);
		GPIOB->CRL |= (GPIO_CRL_MODE6_0 | GPIO_CRL_MODE7_0 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1);

		// Reset I2C1
		I2C1->CR1 |= I2C_CR1_SWRST;
		I2C1->CR1 &= ~I2C_CR1_SWRST;

		// Disable I2C1
		I2C1->CR1 &= ~I2C_CR1_PE;

		// Set I2C1 speed to standard mode 100kHz with 36MHz clock frequency
		I2C1->CR2 = 16; // Set clock frequency to 36 MHz
		I2C1->CCR = 80; // Set clock control to 180 (36MHz / 200kHz)
		I2C1->TRISE = 17;
		// Set own 7-bit slave address
		I2C1->OAR1 |= (0x68 << 1);

		// Enable I2C1 event and buffer interrupts
		I2C1->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);

		// Enable I2C1
		I2C1->CR1 |= I2C_CR1_PE;

		// Enable Acknowledge
		I2C1->CR1 |= I2C_CR1_ACK;
}

volatile uint8_t receivedData = 0;

void I2C1_EV_IRQHandler(void)
{
		I2C1->CR1 |= I2C_CR1_ACK;
    // Check if address matched (slave mode)
    if(I2C1->SR1 & I2C_SR1_ADDR)
    {
        // Clear ADDR flag by reading SR1 and SR2
        (void)I2C1->SR1;
        (void)I2C1->SR2;
    }

    // Check if data received
    if(I2C1->SR1 & I2C_SR1_RXNE)
    {
        // Read received data
        receivedData = I2C1->DR;
    }
}

int main(void)
{
    I2C1_Init();
    while(1)
    {
        
    }
}