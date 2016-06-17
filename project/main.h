#ifndef __MAIN
#define __MAIN value

#include "../include/USER/conf.h"
#include <stdio.h>
#include <math.h>

/* Exported constants --------------------------------------------------------*/
#define SYSCLK_FREQ_72MHz
#define _DLIB_PRINTF_SPECIFIER_FLOAT

#define  USARTx                   USART2                  // seriel port 2
#define  GPIO_RxPin               GPIO_Pin_3              // out port
#define  GPIO_TxPin               GPIO_Pin_2              // in port
#define  GPIOx                    GPIOA
#define  RCC_APB2Periph_GPIOx     RCC_APB2Periph_GPIOA
#define  I2C_Speed                400000
#define  I2C1_SLAVE_ADDRESS7      0xA0
#define  I2C_PageSize             8


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

uint8_t UsartPut(uint8_t ch);
uint8_t UsartGet(void);

#endif /* ifndef __MAIN */
