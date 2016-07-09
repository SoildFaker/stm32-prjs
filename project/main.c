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
  Motor_init();
  Set_pid_gain_value();

  uint8_t a = 0;
  uint16_t dt = 0;

  while (1) {
    dt = TIM3->CNT;
    TIM3->CNT = 0;
    loop_time = (dt*1e-6);
    mpu6050_get_value();
    get_acc_value();
    get_gyro_rate();
    MadgwickAHRSupdateIMU(gyro_x_rate*M_PI/180,gyro_y_rate*M_PI/180,gyro_z_rate*M_PI/180,
                      acc_x_temp,acc_y_temp,acc_z_temp);
    getRollPitchYaw();
    Get_pid_gain_value();//get pid gain value from Processing if requested
    PID_x_update();
    PID_y_update();
    PID_z_update();
    MORTOR_output();
		//printf("running\r");
  /*TIM4->CCR4=2500;*/
    a++;
    if ( a == 0 ){
      myprintf("height:%f\tdt:%d\r\n", HCSR04_Get(), dt);
        myprintf("C1:%d\tC2:%d\tC3:%d\tC4:%d\r\n", TIM4->CCR1, TIM4->CCR2, TIM4->CCR3, TIM4->CCR4);
      myprintf("roll:%f\tpitch:%f\tyaw:%f\r\n", rpy[0], rpy[1], rpy[2]);
    }
  }
  return 0;
}
