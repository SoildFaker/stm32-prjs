/**
 * STM32F103C8 Quadrocopter
 */
#include "tools.h"
#include "mpu6050.h"
#include "motor.h"
#include "nvic.h"

/* Exported constants --------------------------------------------------------*/
uint8_t State = 0;
int main(void)
{
  SystemInit();
  UserInit();
  DelayInit(72);
  
  MPU6050_Init();
  Motor_Init();

  while (1) {
        myprintf("height:%f\r\n", HCSR04_Get() );
        myprintf("C1:%d\tC2:%d\tC3:%d\tC4:%d\r\n", TIM4->CCR1, TIM4->CCR2, TIM4->CCR3, TIM4->CCR4);
        myprintf("roll:%f\tpitch:%f\tyaw:%f\r\n", rpy[0], rpy[1], rpy[2]);
        
  }
  return 0;
}
