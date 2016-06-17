/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* Includes ------------------------------------------------------------------*/
#include "../include/STM32/stm32f10x.h"
//#include "../include/STM32/adc.h"
//#include "../include/STM32/bkp.h"
//#include "../include/STM32/can.h"
//#include "../include/STM32/cec.h"
//#include "../include/STM32/crc.h"
//#include "../include/STM32/dac.h"
//#include "../include/STM32/dbgmcu.h"
#include "../include/STM32/dma.h"
//#include "../include/STM32/fsmc.h"
#include "../include/STM32/i2c.h"
//#include "../include/STM32/iwdg.h"
//#include "../include/STM32/pwr.h"
//#include "../include/STM32/rtc.h"
//#include "../include/STM32/sdio.h"
//#include "../include/STM32/spi.h"
//#include "../include/STM32/wwdg.h"
#include "../include/STM32/tim.h"
#include "../include/STM32/int.h"
#include "../include/STM32/rcc.h"
#include "../include/STM32/gpio.h"
#include "../include/STM32/exti.h"
#include "../include/STM32/flash.h"
#include "../include/STM32/usart.h"
#include "../include/STM32/misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */
#include "../include/DMP/dmpKey.h"
#include "../include/DMP/dmpmap.h"
#include "../include/DMP/inv_mpu.h"
#include "../include/DMP/inv_mpu_dmp_motion_driver.h"
#include "../include/MPU6050/mpu6050.h"



/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */


#endif /* __CONF_H */

