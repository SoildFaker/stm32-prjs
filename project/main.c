/**
 * STM32F103C8 Quadrocopter
 */
#include "tools.h"
#include "conf.h"
#include "MPU6050.h"

/* Exported constants --------------------------------------------------------*/


int main(void)
{
  SystemInit();
  UserInit();
  MPU6050_Init();
  DMP_Init();


  while (1) {  
    printf("height:%d\r\n", HCSR04_Get());
    Read_DMP();
    printf("string:%s\r\n", "Hello World.");
    printf("quad[0]:%d\r\n", iPitch);
    DelayMs(10000);
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
