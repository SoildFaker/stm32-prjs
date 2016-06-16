/**
 * STM32F103C8 Quadrocopter
 */
#include "main.h"

int main(void)
{
  RCC_Conf();
  NVIC_Conf();
  GPIO_Conf();
  USART_Conf();  
  TIMER_Conf();

  IIC_Init();
  MPU6050_Init();
  DMP_Init();

  while (1) {  
    Read_DMP();
    delay_ms(1000);
  }
}

#ifdef  DEBUG
void assert_failed(u8* file, u32 line)
{ 
  while (1)
  {
  }
}
#endif
