/**
 * STM32F103C8 Quadrocopter
 */
#include "tools.h"
#include "conf.h"
#include "MPU6050.h"

/* Exported constants --------------------------------------------------------*/


int main(void)
{
  int16_t AccelGyro[8];
  SystemInit();
  UserInit();


  while (1) {
    MPU6050_GetRawAccelGyro(AccelGyro);
    printf("height:%d\r\n", HCSR04_Get());
    printf("string:%s\r\n", "Hello World.");
    printf("quad[0]:%d\r\n", AccelGyro[0]);
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
