/**
 * STM32F103C8 Quadrocopter
 */
#include "tools.h"
#include "conf.h"
#include "MPU6050.h"

/* Exported constants --------------------------------------------------------*/

int main(void)
{
  float AccelGyro[6];
  SystemInit();
  UserInit();
  DelayInit(72);

  while (1) {
    getAttitude(AccelGyro);
    myprintf("height:%f\r\n", HCSR04_Get());
    myprintf("accel:%f\t%f\t%f\r\n", AccelGyro[0], AccelGyro[1], AccelGyro[2]);
    myprintf("anglr:%f\t%f\t%f\r\n", AccelGyro[3], AccelGyro[4], AccelGyro[5]);
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
