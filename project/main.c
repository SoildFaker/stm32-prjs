/**
 * STM32F103C8 Quadrocopter
 */
#include "tools.h"
#include "ahrs.h"
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

  AHRS_Init();
  Motor_Init();

  uint8_t stop = 0;
  uint8_t a = 0;
  uint8_t b = 0;
  float c = rx_value[2];

  TIM_Cmd(TIM3, ENABLE);//主更新
  rx_value[2] = 0.3f; 
  while (1) {
    if (stop == 0){
      keepHeight(33.0f);
      a = tim3_count-b;
      b = tim3_count;
      myprintf("height:%f\trx:%f\tcount1:%d\r\n", HCSR04_Get(), c, a);
      myprintf("C1:%d\tC2:%d\tC3:%d\tC4:%d\r\n", TIM4->CCR1, TIM4->CCR2, TIM4->CCR3, TIM4->CCR4);
      myprintf("roll:%f\tpitch:%f\tyaw:%f\r\n", rpy[0], rpy[1], rpy[2]);
      myprintf("magx:%f\tmagy:%f\tmagz:%f\r\n", Eangle[0], Eangle[1], Eangle[2]);
      DelayMs(1000);
      if (tim3_count>400){
        stop = 0;
      }
    }else{
      rx_value[2] -= 0.0005f; 
    }
  }
  return 0;
}
