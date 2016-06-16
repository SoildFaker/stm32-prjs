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
/*******************************************************************************
* Function Name  : assert_failed
* Description  : Reports the name of the source file and the source line number
*          where the assert_param error has occurred.
* Input      : - file: pointer to the source file name
*          - line: assert_param error line source number
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
