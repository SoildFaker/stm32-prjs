/**
 * STM32F103C8 Quadrocopter
 */
#include "tools.h"
#include <stdio.h>
#include <math.h>

/* Exported constants --------------------------------------------------------*/
#define SYSCLK_FREQ_72MHz
#define _DLIB_PRINTF_SPECIFIER_FLOAT


int main(void)
{
  SystemInit();
  RCC_Conf();
  NVIC_Conf();
  GPIO_Conf();
  USART_Conf();  
  TIMER_Conf();
  delay_init(72);

  /*IIC_Init();*/
  /*MPU6050_Init();*/
  /*DMP_Init();*/

  double test = 90.323f;

  while (1) {  
    /*Read_DMP();*/
    /*printf("float:%f\r\n", 34.55f);*/
    printf("height:%d\r\n", HCSR04_Get());
    
    delay_us(1000);
    printf("string:%s\r\n", "Hello World.");
    printf("float:%lf\r\n", test);
    delay_ms(1000);
  }
  return 0;
}


#ifdef  DEBUG
void assert_failed(u8* file, u32 line)
{ 
  while (1)
  {
  }
}
#endif
