/**
 * STM32F103C8 Quadrocopter
 */
#include "tools.h"
#include "mpu6050.h"
#include "motor.h"
#include "pid.h"

/* Exported constants --------------------------------------------------------*/
uint8_t State = 0;
int main(void)
{
  SystemInit();
  UserInit();
  DelayInit(72);

  MPU6050_Init();
  Motor_Init();

  uint8_t a = 0;
  uint8_t b = 0;
  uint16_t dt = 0;
  float c = rx_value[2];
  float height_temp = 0.0f;

  while (1) {
    dt = TIM3->CNT;
    TIM3->CNT = 0;
    State_Update();             // 状态信息更新 Pitch, Roll, Yaw , etc.
    PID_Update((float)dt*1e-6);        // PID控制输入每次循环时间
    MORTOR_Output();
    c = rx_value[2];
    a++;
    if ( a == 0 ){
      height_temp = HCSR04_Get();
      if (height_temp > 0 && height_temp < 1000.0f){
        keepHeight(height_temp);
      }else{
        rx_value[2] -= 0.01f; 
      }
      myprintf("height:%f\trx:%f\r\n", height_temp, c);
      myprintf("C1:%d\tC2:%d\tC3:%d\tC4:%d\r\n", TIM4->CCR1, TIM4->CCR2, TIM4->CCR3, TIM4->CCR4);
      myprintf("roll:%f\tpitch:%f\tyaw:%f\r\n", rpy[0], rpy[1], rpy[2]);
      /*b++;*/
      if (b>5){
        Motor_Init();
         while(1);
      }
    }
  }
  return 0;
}
