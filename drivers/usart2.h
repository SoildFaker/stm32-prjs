#ifndef __USART2
#define __USART2 value
#include "main.h"

#define  GPIO_RxPin               GPIO_Pin_3              // out port
#define  GPIO_TxPin               GPIO_Pin_2              // in port

void USART_Conf(void);
uint8_t UsartPut(uint8_t ch);
uint8_t UsartGet(void);
#endif /* ifndef __USART2 */
