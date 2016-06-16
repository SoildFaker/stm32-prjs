#ifndef _LIB
#define _LIB value

#include "conf.h"

/* Exported constants --------------------------------------------------------*/
#define  USARTx                   USART2
#define  GPIOx                    GPIOA
#define  RCC_APB2Periph_GPIOx     RCC_APB2Periph_GPIOA
#define  GPIO_RxPin               GPIO_Pin_3
#define  GPIO_TxPin               GPIO_Pin_2

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

#endif /* ifndef _LIB */
