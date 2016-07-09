#ifndef __TOOLS
#define __TOOLS value

#include "stm32f10x.h"

float HCSR04_Get(void);
void State_Update(void);
void PID_Update(float time);
uint8_t UsartPut(uint8_t ch);
void getAttitude(float* AccelGyro);
uint8_t UsartGet(void);
void myprintf( const char *format, ... );
void print_int( int num, int mode, int flag );
void print_str( char const *str );
void print_float( float num );
void DelayInit(u8 SYSCLK);
void DelayUs(u32 nus);
void DelayMs(u16 nms);
void UserInit(void);
#endif /* ifndef __TOOLS */
