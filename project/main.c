/**
 * STM32F103C8 Quadrocopter
 */
#include "tools.h"
#include "conf.h"

/* Exported constants --------------------------------------------------------*/

int main(void)
{
  SystemInit();
  UserInit();
  DelayInit(72);
  IIC_Init();
  MPU6050_Init();
  DMP_Init();

  while (1) {
    Read_DMP();
    //ahrs computing ---------------          
    myprintf("height:%f\r\n", HCSR04_Get());
    myprintf("roll:%f\tpitch:%f\tyaw:%f\r\n", Roll, Pitch, Yaw);
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
