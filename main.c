#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32l476xx.h"
#include "SysClock.h"
#include "ssd1306.h"
#include "I2C.h"
#include "motor.c"

#define LS 15 // PB15, D6
#define MS 14 // PB14, D5
#define RS 13 // PB13, D4

#define LLED 14
#define RLED 15
#define BUZZ 13

// sensor must read above this value to be considered "on the line"
#define LINE_THRESHOLD 13000
int speed = 430;

void GPIO_init(void)
{
	// enable the cock to GPIO Port A and B
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
	
	// set MODER to alternate function mode for the two LMOTOR pins
	GPIOA->MODER &= ~((0b11 << LMOTOR_FWD*2) | (0b11 << LMOTOR_REV*2));
	GPIOA->MODER |= (0b10 << LMOTOR_FWD*2) | (0b10 << LMOTOR_REV*2);
	
	// for PA5 and PA1, "0001" maps to timer 2 channels 1 and 2, respectively
	GPIOA->AFR[0] |= (1 << LMOTOR_FWD*4) | (1 << LMOTOR_REV*4);
	
	// set MODER to alternate function mode for the two RMOTOR pins
	GPIOA->MODER &= ~((0b11 << RMOTOR_FWD*2) | (0b11 << RMOTOR_REV*2));
	GPIOA->MODER |= (0b10 << RMOTOR_FWD*2) | (0b10 << RMOTOR_REV*2);
	
	// for PA6 and PA7, "0010" maps to timer 3 channels 1 and 2, respectively
	GPIOA->AFR[0] |= (2 << RMOTOR_FWD*4) | (2 << RMOTOR_REV*4);
	
	// set MODER for LED pins and buzzer
	GPIOA->MODER &= ~(0b11 << 2*LLED);
	GPIOA->MODER |= (0b01 << 2*LLED);
	GPIOA->MODER &= ~(0b11 << 2*RLED);
	GPIOA->MODER |= (0b01 << 2*RLED);
	GPIOA->MODER &= ~(0b11 << 2*BUZZ);
	GPIOA->MODER |= (0b01 << 2*BUZZ);
	
	// set output low for buzzer
	GPIOA->ODR &= ~(1 << BUZZ);
}

int readSensor(int pin)
{
	int counter = 0;
	GPIOB->ODR &= ~(1 << pin);
	
	// output
	GPIOB->MODER &= ~(0b11 << 2*pin);
	GPIOB->MODER |= (0b01 << 2*pin);
	GPIOB->ODR |= 1 << pin;
	
	delayMs(1);
	
	// input
	GPIOB->MODER &= ~(0b11 << 2*pin);
	
	// wait for pin to go low
	while (GPIOB->IDR & (1 << pin) && counter < LINE_THRESHOLD + 1) counter++;
	
	return (counter > LINE_THRESHOLD);
}

void print(char* str, int cu, int line) 
{
	// cu is "clear-update"
	// if cu is 0, just print
	// if cu is 1, clear before printing
	// if cu is 2, update screen after printing
	if (cu == 1) ssd1306_Fill(Black);
	ssd1306_SetCursor(0,10*line);
	ssd1306_WriteString(str, Font_7x10, White);
	if (cu == 2) ssd1306_UpdateScreen();
}

int state = 1;
void printSensorVals()
{
	char ls[10];
	char ms[10];
	char rs[20];
	sprintf(ls, "L %d", readSensor(LS));
	sprintf(ms, "M %d", readSensor(MS));
	sprintf(rs, "R %d, %s", readSensor(RS), (state) ? "state" : "left");
	print(ls, 1, 2);
	print(ms, 0, 3);
	print(rs, 2, 4);
}
void switchState()
{
	// four possible values:
	// 0 = on the line. MS = black, RS = LS = white
	if (state == 1) state = 0;
	else state = 1;
}

int main(void)
{
	System_Clock_Init(); 
	I2C_GPIO_init();
	I2C_Initialization(I2C1);
	ssd1306_Init();
	ssd1306_SetCursor(2,0);
	GPIO_init();
	TIM2_init();
	TIM3_init();

	int prevState = state;
	int lval, rval;
	int stateCounter;
	while (1)
	{
		
		// update state
		lval = readSensor(LS);
		rval = readSensor(RS);
		GPIOA->ODR &= ~(1 << BUZZ);
		if ((lval && rval) || (!lval && !rval)) continue;
		else if (lval) state = 1;
		else if (rval) state = 2;
		
		// update motors and LEDs
		if (state == 2)
		{
			setLeftMotorSpeed(speed);
			setRightMotorSpeed(0);
			GPIOA->ODR &= ~(1 << LLED);
			GPIOA->ODR |= (1 << RLED);
		}
		else if (state == 1)
		{
			setRightMotorSpeed(speed + 30);
			setLeftMotorSpeed(0);
			GPIOA->ODR &= ~(1 << RLED);
			GPIOA->ODR |= (1 << LLED);
		}
		if (prevState != state) GPIOA->ODR |= (1 << BUZZ);
		prevState = state;
}
}

