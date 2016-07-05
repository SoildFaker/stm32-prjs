/**
 * STM32F103C8 Quadrocopter
 */
#include "tools.h"
#include "conf.h"
#include "MPU6050.h"

/* Exported constants --------------------------------------------------------*/

int main(void)
{
  int16_t AccelGyro[6];
  uint8_t gyroRange, accelRange;
  SystemInit();
  UserInit();
  DelayInit(72);
  gyroRange = MPU6050_GetFullScaleGyroRange();
  accelRange = MPU6050_GetFullScaleAccelRange();

  while (1) {
    getAttitude(AccelGyro);
    printf("height:%d\r\n", HCSR04_Get());
    printf("accel:%d\t%d\t%d\r\n", AccelGyro[0], AccelGyro[1], AccelGyro[2]);
    printf("anglr:%d\t%d\t%d\r\n", AccelGyro[3], AccelGyro[4], AccelGyro[5]);
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
