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
  printf("debug:MPU\r\n");
  MPU6050_Init();
  printf("debug:DMP\r\n");
  DMP_Init();

  printf("debug:Start\r\n");

  while (1) {  
    printf("height:%d\r\n", HCSR04_Get());
    Read_DMP();
    DelayUs(1000);
    printf("string:%s\r\n", "Hello World.");
    printf("float:%lf\r\n", Pitch);
    DelayMs(1000);
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
