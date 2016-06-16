/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONF_H
#define __CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../include/STM32/stm32f10x.h"
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

//#define SYSCLK_HSE
#define SYSCLK_FREQ_24MHz

#if !defined STM32F10X_LD_VL && !defined STM32F10X_MD_VL && !defined STM32F10X_HD_VL
  //#define SYSCLK_FREQ_36MHz
  //#define SYSCLK_FREQ_48MHz
  //#define SYSCLK_FREQ_56MHz
  #define SYSCLK_FREQ_72MHz
#endif

/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif

#endif /* __CONF_H */

