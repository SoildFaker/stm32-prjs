#ifndef __TIME
#define __TIME value
#include "main.h"

void RCC_Conf(void);
void TIMER_Conf(void);
void delay_init(uint8_t SYSCLK);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);
#endif /* ifndef __TIME */
