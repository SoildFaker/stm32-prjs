#ifndef __TOOLS
#define __TOOLS value

#include "stm32f10x.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: 

#define MAG0MAX 603
#define MAG0MIN -578

#define MAG1MAX 542
#define MAG1MIN -701

#define MAG2MAX 547
#define MAG2MIN -556

#define RAD_TO_DEG 57.295779513082320876798154814105  // 弧度转角度的转换率
#define DEG_TO_RAD 0.01745329251994329576923690768489 // 角度转弧度的转换率

extern double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer

float HCSR04_Get(void);
void State_Update(void);
void updatePitchRoll(void);
void updateYaw(void);
void Main_Update(void);
void PID_Update(void);
uint8_t UsartPut(uint8_t ch);
void getAttitude(float* AccelGyro);
uint8_t UsartGet(void);
void myprintf( const char *format, ... );
void print_int( int num, int mode, int flag );
void print_str( char const *str );
void print_float( float num );
void DelayInit(u8 SYSCLK);
int get_tick_count(unsigned long *count);
void DelayUs(u32 nus);
void DelayMs(u16 nms);
void UserInit(void);
#endif /* ifndef __TOOLS */
