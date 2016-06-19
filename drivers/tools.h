#ifndef __TOOLS
#define __TOOLS value

#include "conf.h"

int HCSR04_Get(void);
uint8_t UsartPut(uint8_t ch);
uint8_t UsartGet(void);
void delay_init(u8 SYSCLK);
void delay_us(u32 nus);
void delay_ms(u16 nms);
#endif /* ifndef __TOOLS */
