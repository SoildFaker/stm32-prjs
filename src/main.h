/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../include/STM32/stm32f10x.h"
#include "../include/IIC/IOI2C.h"
#include "../include/DMP/dmpKey.h"
#include "../include/DMP/dmpmap.h"
#include "../include/DMP/inv_mpu.h"
#include "../include/DMP/inv_mpu_dmp_motion_driver.h"
#include "../include/MPU6050/mpu6050.h"
#include "../include/conf.h"
#include <stdio.h>
#include <math.h>

/* Exported types ------------------------------------------------------------*/
u16 counter;
/* Exported functions ------------------------------------------------------- */
void USART_Conf(void);
void RCC_Conf(void);
void GPIO_Conf(void);
void NVIC_Conf(void);
void TIMER_Conf(void);
int HCSR04_Get(void);
void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
/* Exported constants --------------------------------------------------------*/
#define  USARTx                   USART2
#define  GPIOx                    GPIOA
#define  RCC_APB2Periph_GPIOx     RCC_APB2Periph_GPIOA
#define  GPIO_RxPin               GPIO_Pin_3
#define  GPIO_TxPin               GPIO_Pin_2

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

