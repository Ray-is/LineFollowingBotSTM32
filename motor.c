#include "stm32l476xx.h"

#define LMOTOR_FWD 5 // D13
#define LMOTOR_REV 1 // A1
#define RMOTOR_FWD 7 // D11
#define RMOTOR_REV 6 // D12

// note: changing these pins may require changes in the AFR register setting in GPIOA_init

void delayMs(uint16_t ms)
{
	if (ms <= 0 || ms >= 32000) return; // ms can't be greater than maximum 16bit uint
	ms *= 2;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN; // enable timer's clock
	TIM7->CR1 &= ~TIM_CR1_CEN;            // disable the timer's counter
	TIM7->SR = 0;                         // clear status register. this single-bit register is set when CNT reaches ARR.
	TIM7->CNT = 0;                        // start CNT at 0
	TIM7->PSC = (4000 - 1) * 10;          // 80Mhz down to 2Khz
	TIM7->ARR = ms - 1;                   // control how long to count for
	TIM7->CR1 |= TIM_CR1_CEN;             // enable the counter to begin delay period
	while ((TIM7->SR & TIM_SR_UIF) == 0); // wait until CNT reaches ARR, which means the delay period has finished
}

void TIM2_init(void)
{
	// enable the cock to timer 2
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	
	// 80Mhz down to 50 kHz
	TIM2->PSC = (80 - 1) * 20;
	
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
	
	// 80Mhz down to 50 kHz
	TIM3->PSC = (80 - 1) * 20;
	
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

void stop()
{
	setLeftMotorSpeed(0);
	setRightMotorSpeed(0);
}

void straight(int spd)
{
	setLeftMotorSpeed(spd);
	setRightMotorSpeed(spd + 50);
}

void reverse(int spd, int duration)
{
	setLeftMotorSpeed(-spd);
	setRightMotorSpeed(-spd);
	delayMs(duration);
	stop();
	delayMs(400);
}

void testMotors(void)
{	
	// move left motor forward, full speed
	setLeftMotorSpeed(1000);
	delayMs(1000);
	setLeftMotorSpeed(0);
	delayMs(1000);
		
	// move right motor forward, full speed
	setRightMotorSpeed(1000);
	delayMs(1000);
	setRightMotorSpeed(0);
	delayMs(1000);
	
	// move both motors backward, full speed
	setLeftMotorSpeed(-1000);
	setRightMotorSpeed(-1000);
	delayMs(1000);
	setLeftMotorSpeed(0);
	setRightMotorSpeed(0);
	
	// slowly increase the speed of both motors until it's maxed, wait 1 second while at max power, then stop both motors
	int spd = 0;
	while (spd <= 1000)
	{
		setLeftMotorSpeed(spd);
		setRightMotorSpeed(spd);
		spd++;
		delayMs(10);
	}
	delayMs(1000);
	setLeftMotorSpeed(0);
	setRightMotorSpeed(0);
}