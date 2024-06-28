#include "stm32f4xx.h"

#define LCD_ADDR (0x27 << 1)

void delay(uint32_t val)
{
		for (int i = 0; i < val; i++);
}

void I2C1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
    GPIOB->OTYPER |= GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0;
    GPIOB->AFR[1] |= (4 << (4*0)) | (4 << (4*1));

	I2C1->CR1 |= (1<<15);
	I2C1->CR1 &= ~(1<<15);

    I2C1->CR2 |= (45<<0);
    I2C1->CCR = 225<<0;
    I2C1->TRISE = 46;
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C1_SendData(uint8_t address, uint8_t data)
{
	I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;
	
    while(!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = address;
    while(!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR1;
    (void)I2C1->SR2;
	
	while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while(!(I2C1->SR1 & I2C_SR1_BTF));
	
    I2C1->CR1 |= I2C_CR1_STOP;
}

#define LCD_EN 0x04
#define LCD_RS 0x01

void LCD_SendCmd(uint8_t cmd) {
    uint8_t data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd&0xF0);
    data_l = ((cmd<<4)&0xF0);
    data_t[0] = data_u|0x0C;
    data_t[1] = data_u|0x08;
    data_t[2] = data_l|0x0C;
    data_t[3] = data_l|0x08;
    for(int i=0; i<4; i++)
    {
        I2C1_SendData(LCD_ADDR, data_t[i]);
		delay(2);
    }
}

void LCD_SendData(uint8_t data) {
    uint8_t data_u, data_l;
    uint8_t data_t[4];
    data_u = (data&0xF0);
    data_l = ((data<<4)&0xF0);
    data_t[0] = data_u|0x0D;
    data_t[1] = data_u|0x09;
    data_t[2] = data_l|0x0D;
    data_t[3] = data_l|0x09;
    for(int i=0; i<4; i++)
    {
        I2C1_SendData(LCD_ADDR, data_t[i]);
        delay(2);
    }
}

void LCD_Init (void)
{
	// 4 bit initialisation
	delay(50000);  // wait for >40ms
	LCD_SendCmd (0x30);
	delay(50000);  // wait for >4.1ms
	LCD_SendCmd (0x30);
	delay(1000);  // wait for >100us
	LCD_SendCmd (0x30);
	delay(10000);
	LCD_SendCmd (0x20);  // 4bit mode
	delay(10000);

  // dislay initialisation
	LCD_SendCmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	delay(1000);
	LCD_SendCmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	delay(1000);
	LCD_SendCmd (0x01);  // clear display
	delay(1000);
	delay(1000);
	LCD_SendCmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	delay(1000);
	LCD_SendCmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void LCD_PutStr(char *str) {
    while (*str) {
        LCD_SendData((uint8_t)(*str));
        str++;
    }
}

int main(void)
{
    I2C1_Init();
    LCD_Init();

    while(1)
    {
        LCD_PutStr("T");
        delay(1000);
        LCD_SendCmd(0x01);
        delay(1000);
    }
}