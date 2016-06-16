/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "../include/STM32/adc.h"
//#include "../include/STM32/bkp.h"
//#include "../include/STM32/can.h"
//#include "../include/STM32/cec.h"
//#include "../include/STM32/crc.h"
//#include "../include/STM32/dac.h"
//#include "../include/STM32/dbgmcu.h"
//#include "../include/STM32/dma.h"
//#include "../include/STM32/fsmc.h"
//#include "../include/STM32/i2c.h"
//#include "../include/STM32/iwdg.h"
//#include "../include/STM32/pwr.h"
//#include "../include/STM32/rtc.h"
//#include "../include/STM32/sdio.h"
//#include "../include/STM32/spi.h"
//#include "../include/STM32/wwdg.h"
#include "../include/STM32/tim.h"
#include "../include/STM32/it.h"
#include "../include/STM32/rcc.h"
#include "../include/STM32/gpio.h"
#include "../include/STM32/exti.h"
#include "../include/STM32/flash.h"
#include "../include/STM32/usart.h"
#include "../include/STM32/misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */
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

