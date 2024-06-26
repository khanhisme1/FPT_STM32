#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>

void USART2_Transmit(char c)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void USART2_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->CRL |= GPIO_CRL_MODE2 | GPIO_CRL_CNF2_1 | GPIO_CRL_CNF3_0;
    GPIOA->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF2_0 | GPIO_CRL_CNF3_1);

    USART2->BRR = 0x341;
    USART2->CR1 |= USART_CR1_UE;
    USART2->CR1 |= USART_CR1_TE;
}


void Timer2_Init(void) { 
    //Clock enable 
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; 
}

void delay_us(uint32_t us) {
    //Prescaler
    TIM2->PSC = 8 - 1;
    
    //Auto-reload value
    TIM2->ARR = us;
    
    //Forced update
    TIM2->EGR = TIM_EGR_UG;
    
    //Clear update flag
    TIM2->SR &= ~TIM_SR_UIF; 
    
    //Timer enable 
    TIM2->CR1 |= TIM_CR1_CEN; 
    while (!(TIM2->SR & TIM_SR_UIF)); // wait till update flag is set 
    
    //Clear update flag
    TIM2->SR &= ~TIM_SR_UIF;
    
    //Stop timer
    TIM2->CR1 &= ~TIM_CR1_CEN; 
}

void DHT22_Start(void)
{   
    // Configure PA4 as output
    GPIOA->CRL &= ~(GPIO_CRL_CNF4);
    GPIOA->CRL |= GPIO_CRL_MODE4_0;

    // Reset PA4
    GPIOA->ODR &= ~GPIO_ODR_ODR4;
    delay_us(18000);

    // Set PA4
    GPIOA->ODR |= GPIO_ODR_ODR4;
    delay_us(20);
    GPIOA->ODR &= ~GPIO_ODR_ODR4;

    // Set PA4 as input
    GPIOA->CRL &= ~GPIO_CRL_MODE4;
}

uint8_t DHT22_Check_Response (void) {
    uint8_t response = 0;
    delay_us(40);

    //Check if the data is ready to be sent
    if (!(GPIOA->IDR & GPIO_IDR_IDR4))
    {
        delay_us(80);
        if (GPIOA->IDR & GPIO_IDR_IDR4) response = 1;
        else response = 2;
    }
    while (GPIOA->IDR & GPIO_IDR_IDR4); // wait till input is 0

    return response;
}

uint8_t DHT22_Read()
{
    uint8_t data, i;
    //Read 8 bit
    for(i = 0; i < 8; i++)
    {
        while (!(GPIOA->IDR & GPIO_IDR_IDR4));
        delay_us(40);
        if (!(GPIOA->IDR & GPIO_IDR_IDR4))
            data &= ~(1<<(7-i));
        else
        {
            data |= (1<<(7-i));
        }
        while (GPIOA->IDR & GPIO_IDR_IDR4);
    }
    return data;
}

void UART_Transmit_String(char* str)
{
    for (uint8_t i = 0; i < strlen(str); i++)
    {
        USART2_Transmit(str[i]);
    }
}

void UART_Transmit_Float(float number)
{
    char buffer[32];
    sprintf(buffer, "%f", number);
    
    for (uint8_t i = 0; i < strlen(buffer); i++)
    {
        USART2_Transmit(buffer[i]);
    }
}

int main(void)
{
    USART2_Init();
	Timer2_Init();
    uint8_t response;
	uint8_t humidity_first_byte;
	uint8_t humidity_second_byte;
	uint8_t temperature_first_byte;
	uint8_t temperature_second_byte;
	uint8_t sum;
	uint16_t raw_humidity;
	uint16_t raw_temperature;
	float humidity;
	float temperature;

	while (1)
	{
		//		//Dirt sensor
		//		adcValue0 = ADC_Read_Channel_0();
		//		PWM3_SetDutyCycle(adcValue0);
		//
		// Light sensor
		//		adcValue1 = ADC_Read_Channel_1();
		//		PWM3_SetDutyCycle(adcValue1);

		// Humidity and temperature sensor
		DHT22_Start();
		response = DHT22_Check_Response();
		if (response == 2)
		{
			UART_Transmit_String("Error");
		}
		else
		{
			humidity_first_byte = DHT22_Read();
			humidity_second_byte = DHT22_Read();
			temperature_first_byte = DHT22_Read();
			temperature_second_byte = DHT22_Read();
			sum = DHT22_Read();

			raw_humidity = (humidity_first_byte << 8) | humidity_second_byte;
			raw_temperature = (temperature_first_byte << 8) | temperature_second_byte;

			humidity = raw_humidity / 10.0f;
			temperature = raw_temperature / 10.0f;

			//			if (humidity < 70 || temperature > 30) {
			//				GPIOB0_Off();
			//			} else {
			//				GPIOB0_On();
			//			}

			// Send data to LCD
			//			LCD_PutStringFloat("Hudmidity:", humidity, 1);
			//			LCD_PutStringFloat("Temperature:", temperature, 2);

			// Send data to PC
			UART_Transmit_String("Hudmidity: ");
			UART_Transmit_Float(humidity);
			UART_Transmit_String("\nTemperature: ");
			UART_Transmit_Float(temperature);
			UART_Transmit_String("\n\n");
		}
	}
    return 0;
}