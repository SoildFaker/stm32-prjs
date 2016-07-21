/**
 * STM32F103C8 Quadrocopter
 */
#include "tools.h"
#include "ahrs.h"
#include "mpu6050.h"
#include "flow.h"
#include "nvic.h"
#include "pid.h"

/* Exported constants --------------------------------------------------------*/
uint8_t State = 0;
int main(void)
{
  SystemInit();
  UserInit();

  MPU6050_Init();
  ADNS3080_Init();

  uint8_t stop = 0;
  uint8_t dt = 0;
  uint8_t a = 0;

  throttle = 0.3f; 

  delay_ms(10);
  TIM3->CNT = 0;
  while (1) {
    dt = TIM3->CNT;
    TIM3->CNT = 0;
    if (stop == 0){
      keepHeight(33.0f);
      State_Update((float)1e-6*dt);      // 状态信息更新 Pitch, Roll, Yaw , etc.
      PID_Update((float)1e-6*dt);        // PID控制输入每次循环时间
      MORTOR_Output();
      a++;
      if (a == 0){
        myprintf("height:%f\tthrottle:%f\tdt:%d\r\n", HCSR04_Get(), throttle, dt);
        myprintf("C1:%d\tC2:%d\tC3:%d\tC4:%d\r\n", TIM4->CCR1, TIM4->CCR2, TIM4->CCR3, TIM4->CCR4);
        myprintf("roll:%f\tpitch:%f\tyaw:%f\r\n", roll, pitch, yaw);
        Read_Data_burst();
      }
      if (tim3_count>400){
        stop = 0;
      }
    }else{
      rx_value[2] -= 0.0005f; 
    }
  }
  return 0;
}
