/**
 * STM32F103C8 Quadrocopter
 */
#include "tools.h"
#include "mpu6050.h"
#include "motor.h"
#include "nvic.h"
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

  uint8_t stop = 0;
  uint8_t a = 0;
  uint16_t dt = 0;
  float c = rx_value[2];

  TIM3->CNT = 0;
  rx_value[2] = 0.3f; 
  while (1) {
    a++;
        myprintf("height");
    if (stop == 0){
      if(a%10000==0){
        keepHeight(33.0f);
        myprintf("height:%f\trx:%f\tcount1:%d\r\n", HCSR04_Get(), c, tim3_count);
        myprintf("C1:%d\tC2:%d\tC3:%d\tC4:%d\r\n", TIM4->CCR1, TIM4->CCR2, TIM4->CCR3, TIM4->CCR4);
        myprintf("roll:%f\tpitch:%f\tyaw:%f\r\n", rpy[0], rpy[1], rpy[2]);
      }
      if (tim3_count>400){
        stop = 1;
      }
    }else{
      rx_value[2] -= 0.0005f; 
    }
  }
  return 0;
}
