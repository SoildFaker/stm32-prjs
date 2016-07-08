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

  mpu_6050_init();

  uint8_t a = 0;

  uint16_t dt = 0;

  while (1) {
    dt = TIM3->CNT;
    TIM3->CNT = 0;
    mpu6050_get_value();
    get_acc_value();
    get_gyro_rate();
    MadgwickAHRSupdateIMU(gyro_x_rate*M_PI/180,gyro_y_rate*M_PI/180,gyro_z_rate*M_PI/180,
                      acc_x_temp,acc_y_temp,acc_z_temp);
    getRollPitchYaw();
    a++;
    if ( a == 0 ){
      myprintf("height:%f\tdt:%d\r\n", HCSR04_Get(), dt);
      myprintf("roll:%f\tpitch:%f\tyaw:%f\r\n", rpy[0], rpy[1], rpy[2]);
    }
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
