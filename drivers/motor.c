#include "motor.h"
#include "nvic.h"

void Motor_Disabled(void)
{
	TIM4->CCR1=0;
	TIM4->CCR2=0;
	TIM4->CCR3=0;
	TIM4->CCR4=0;
}

void Motor_init(void)
{
	TIM4->CCR1=STOP_PWM;
	TIM4->CCR2=STOP_PWM;
	TIM4->CCR3=STOP_PWM;
	TIM4->CCR4=STOP_PWM;
}

void Motor_Start(void)
{
  uint16_t counter = tim4_count;
  
}
