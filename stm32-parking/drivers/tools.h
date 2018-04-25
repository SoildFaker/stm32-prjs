#ifndef __TOOLS
#define __TOOLS value

#include "stm32f10x.h"
extern float height;

float HCSR04_Get(void);
void State_Update(float dt);
void PWM_Init(void);
void PID_Update(float dt);
uint8_t UsartPut(uint8_t ch);
void getAttitude(float* AccelGyro);
uint8_t UsartGet(void);
void myprintf( const char *format, ... );
void print_int( int num, int mode, int flag );
void print_str( char const *str );
void print_float( float num );
void DelayInit(u8 SYSCLK);
int get_tick_count(unsigned long *count);
void delay_us(u32 nus);
void delay_ms(u16 nms);
void UserInit(void);
#endif /* ifndef __TOOLS */
