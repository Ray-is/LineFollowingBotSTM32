#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32l476xx.h"
#include "SysClock.h"

#define LMOTOR_FWD 5 // D13
#define LMOTOR_REV 1 // A1
#define RMOTOR_FWD 7 // D11
#define RMOTOR_REV 6 // D12
// note: changing these pins may require changes in the AFR register setting in GPIOA_init

void myDelay(uint16_t ms)
{
	if (ms == 0) return;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN; // enable timer's clock
	TIM7->CR1 &= ~TIM_CR1_CEN;            // disable the timer's counter
	TIM7->SR = 0;                         // clear status register. this single-bit register is set when CNT reaches ARR.
	TIM7->CNT = 0;                        // start CNT at 0
	TIM7->PSC = 4000 - 1;                 // 4Mhz down to 1Khz
	TIM7->ARR = ms - 1;                   // control how long to count for
	TIM7->CR1 |= TIM_CR1_CEN;             // enable the counter to begin delay period
	while ((TIM7->SR & TIM_SR_UIF) == 0); // wait until CNT reaches ARR, which means the delay period has finished
}

void TIM2_init(void)
{
	// enable the cock to timer 2
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	
	// 4Mhz down to 50 kHz
	TIM2->PSC = 80 - 1;
	
	// pwm period = 20ms, 1000 discrete dutycycles
	TIM2->ARR = 1000 - 1;
	
	// initial dutycycle = 0%
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	
	// set PWM mode 1 for both channels
	TIM2->CCMR1 |= (0b110  << 4) | (0b110 << 12);
	
	// enable preload for ch1 and 2. preload ensures that changes to CCR1 only take effect on the next PWM cycle.
	TIM2->CCMR1 |= (1 << 3) | (1 << 11);
	
	// enable channel 1 output and channel 2 output
	TIM2->CCER |= 1 | (1 << 4);
	
	// switch polarity, active low
	TIM2->CCER |= (1 << 1) | (1 << 5);
	
	// global enable
	TIM2->BDTR |= TIM_BDTR_MOE;
	
	// enable counter
	TIM2->CR1 |= 1;
}

void TIM3_init(void)
{
	// enable the cock to timer 2
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
	
	// 4Mhz down to 50 kHz
	TIM3->PSC = 80 - 1;
	
	// pwm period = 20ms, 1000 discrete dutycycles
	TIM3->ARR = 1000 - 1;
	
	// initial dutycycle = 0%
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	
	// set PWM mode 1 for both channels
	TIM3->CCMR1 |= (0b110  << 4) | (0b110 << 12);
	
	// enable preload for ch1 and 2. preload ensures that changes to CCR1 only take effect on the next PWM cycle.
	TIM3->CCMR1 |= (1 << 3) | (1 << 11);
	
	// enable channel 1 output and channel 2 output
	TIM3->CCER |= 1 | (1 << 4);
	
	// switch polarity, active low
	TIM3->CCER |= (1 << 1) | (1 << 5);
	
	// global enable
	TIM3->BDTR |= TIM_BDTR_MOE;
	
	// enable counter
	TIM3->CR1 |= 1;
}

void setLeftMotorSpeed(int ds) // ds is from -1000 to 1000
{	
	
	// forward
	if (ds < 0)
	{
		TIM2->CCR1 = -ds;
		TIM2->CCR2 = 0;
	}
	
	// reverse
	else if (ds > 0)
	{
		TIM2->CCR1 = 0;
		TIM2->CCR2 = ds;
	}
	
	// neutral, ds=0
	else
	{
		TIM2->CCR1 = 0;
		TIM2->CCR2 = 0;
	}
	
}

void setRightMotorSpeed(int ds) // ds is from -1000 to 1000
{	
	
	// forward
	if (ds < 0)
	{
		TIM3->CCR1 = -ds;
		TIM3->CCR2 = 0;
	}
	
	// reverse
	else if (ds > 0)
	{
		TIM3->CCR1 = 0;
		TIM3->CCR2 = ds;
	}
	
	// neutral, ds=0
	else
	{
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
	}
	
}

void GPIOA_init(void)
{
	// enable the cock to GPIO Port A
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	
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
	
	
}

void testMotors(void)
{	
	// move left motor forward, full speed
	setLeftMotorSpeed(1000);
	myDelay(1000);
	setLeftMotorSpeed(0);
	myDelay(1000);
		
	// move right motor forward, full speed
	setRightMotorSpeed(1000);
	myDelay(1000);
	setRightMotorSpeed(0);
	myDelay(1000);
	
	// move both motors backward, full speed
	setLeftMotorSpeed(-1000);
	setRightMotorSpeed(-1000);
	myDelay(1000);
	setLeftMotorSpeed(0);
	setRightMotorSpeed(0);
	
	// slowly increase the speed of both motors until it's maxed, wait 1 second while at max power, then stop both motors
	int spd = 0;
	while (spd <= 1000)
	{
		setLeftMotorSpeed(spd);
		setRightMotorSpeed(spd);
		spd++;
		myDelay(10);
	}
	myDelay(1000);
	setLeftMotorSpeed(0);
	setRightMotorSpeed(0);
}





int main(void)
{
	int spd = 0;
	int spdStep = 1;
	
	GPIOA_init();
	TIM2_init();
	TIM3_init();
	
	while (1)
	{
		myDelay(5000);
		testMotors();

	}
}

