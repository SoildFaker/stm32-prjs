#include "motor.h"
#include "stm32f10x_tim.h"
#include "pid.h"

void Motor_Disabled(void)
{
	TIM4->CCR1=0;
	TIM4->CCR2=0;
	TIM4->CCR3=0;
	TIM4->CCR4=0;
}

void Motor_Init(void)
{
	TIM4->CCR1=STOP_PWM;
	TIM4->CCR2=STOP_PWM;
	TIM4->CCR3=STOP_PWM;
	TIM4->CCR4=STOP_PWM;

  PID_SetGainValue();
}

