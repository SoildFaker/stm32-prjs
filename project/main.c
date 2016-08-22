/**
 * STM32F103C8 Quadrocopter
 */
#include "tools.h"
#include <stdio.h>
#include "ahrs.h"
/*#include "magnet.h"*/
#include "mpu6050.h"
#include "flow.h"
#include "nvic.h"
#include "conf.h"
#include "pid.h"
/* Exported constants --------------------------------------------------------*/
uint8_t stop = 0;
int main(void)
{
  SystemInit();
  UserInit();

  MPU6050_Init();
  ADNS3080_Init();
  /*MAG3110_Init();*/

  uint8_t dt = 0;
  uint16_t idt = 0;
  uint8_t a = 0;
  uint8_t b = 0;

  throttle = 0.3f; 

  delay_ms(10);
  TIM3->CNT = 0;
  while (1) {
    dt = TIM3->CNT;
    TIM3->CNT = 0;
    if (stop == 0){
      State_Update((float)1e-6*dt);      // 状态信息更新 Pitch, Roll, Yaw , etc.
      PID_Update((float)1e-6*dt);        // PID控制输入每次循环时间
      MORTOR_Output();
      /*PIDxp_Update((float)1e-6*dt);*/
      /*PIDyp_Update((float)1e-6*dt);*/
      a++;
      idt+=dt;
      if (a%200 == 0){
        height = HCSR04_Get();
        
        PIDzp_Update((float)1e-6*idt);
        /*myprintf("============================================================\r\n");*/
        /*myprintf("height:%f\tthrottle:%f\tdt:%d\tidt:%d\r\n", HCSR04_Get(), throttle, dt, idt);*/
        /*myprintf("C1:%d\tC2:%d\tC3:%d\tC4:%d\r\n", TIM4->CCR1, TIM4->CCR2, TIM4->CCR3, TIM4->CCR4);*/
        /*myprintf("roll:%f\tpitch:%f\tyaw:%f\r\n", roll, pitch, yaw);*/
        /*myprintf("r:%f\tp:%f\r\n", rx_value[0], rx_value[1]);*/
        /*myprintf("X:%d\tY:%d\tZ:%f\r\n", X, Y, height);*/
        /*myprintf("TIM3:%d\r\n",tim3_count);*/
        idt=0;
        b++;
      }
      if (b>30){
        stop = 1;
      }
    }else{
      throttle -= 0.0005f; 
      State_Update((float)1e-6*dt);      // 状态信息更新 Pitch, Roll, Yaw , etc.
      PID_Update((float)1e-6*dt);        // PID控制输入每次循环时间
      MORTOR_Output();
    }
  }
  return 0;
}
