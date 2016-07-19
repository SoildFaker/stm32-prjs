#include "tools.h"
#include "mpu6050.h"
#include "kalman.h"
#include "mag3110.h"
#include "stm32f10x_usart.h"
#include "stm32f10x.h"
#include "conf.h"
#include <stdarg.h>  
#include <math.h>

struct Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances
double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only 只用陀螺仪计算角度
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter  用电磁计计算角度
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter  用kalman计算角度

//uint32_t timer,micros; //上一次时间与当前时间
uint8_t i2cData[14]; // Buffer for I2C data
float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };
double magGain[3];


static u8  fac_us=0;//us延时倍乘数
static u16 fac_ms=0;//ms延时倍乘数

extern uint16_t timer_counter;

va_list args;  
  
char sign[] = { '0','1','2','3','4','5',  
    '6','7','8','9','a','b',  
    'c','d','e','f'};  
 
void UserInit(void)
{
  RCC_Conf();
  GPIO_Conf();
  TIMER_Conf();
  PWM_Conf();
  USART_Conf();
  /*I2C_Conf();*/
  NVIC_Conf();
}

void Main_Update(void)
{
  double gyroXrate,gyroYrate,gyroZrate,dt=0.01;
  /* Update all the IMU values */
  MPU6050_Update();
  MAG3110_Update();
  
//  dt = (double)(micros - timer) / 1000; // Calculate delta time
//  timer = micros;
//  if(dt<0)dt+=(1<<20);  //时间是周期性的，有可能当前时间小于上次时间，因为这个周期远大于两次积分时间，所以最多相差1<<20

  /* Roll and pitch estimation */
  updatePitchRoll();       //用采集的加速计的值计算roll和pitch的值
  gyroXrate = gyroX / 131.0;   // Convert to deg/s  把陀螺仪的角加速度按照当初设定的量程转换为°/s
  gyroYrate = gyroY / 131.0;   // Convert to deg/s
  
  #ifdef RESTRICT_PITCH    //如果上面有#define RESTRICT_PITCH就采用这种方法计算，防止出现-180和180之间的跳跃
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    setAngle(&kalmanX,roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
  kalAngleX = getAngle(&kalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  
  if (fabs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleY = getAngle(&kalmanY,pitch, gyroYrate, dt);
  #else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
  kalAngleY = getAngle(&kalmanY, pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  
  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleX = getAngle(&kalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif
  
  
  /* Yaw estimation */
  updateYaw();
  gyroZrate = gyroZ / 131.0; // Convert to deg/s
  // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
  if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
    setAngle(&kalmanZ,yaw);
    compAngleZ = yaw;
    kalAngleZ = yaw;
    gyroZangle = yaw;
  } else
  kalAngleZ = getAngle(&kalmanZ, yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter
  
  
  /* Estimate angles using gyro only */
  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate from the Kalman filter
  //gyroYangle += kalmanY.getRate() * dt;
  //gyroZangle += kalmanZ.getRate() * dt;
  
  /* Estimate angles using complimentary filter */
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;
  
  // Reset the gyro angles when they has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;
  
  
  /*send(roll,pitch,yaw);*/
//  send(gyroXangle,gyroYangle,gyroZangle);
//  send(compAngleX,compAngleY,compAngleZ);
//  send(kalAngleX,kalAngleY,kalAngleZ);
//  send(kalAngleY,compAngleY,gyroYangle);


  /* Print Data */
//  //#if 1
//  printf("%lf %lf %lf %lf\n",roll,gyroXangle,compAngleX,kalAngleX);
//  printf("%lf %lf %lf %lf\n",pitch,gyroYangle,compAngleY,kalAngleY);
//  printf("%lf %lf %lf %lf\n",yaw,gyroZangle,compAngleZ,kalAngleZ);
  //#endif
  
//  //#if 0 // Set to 1 to print the IMU data
//  printf("%lf %lf %lf\n",accX / 16384.0,accY / 16384.0,accZ / 16384.0);
//  printf("%lf %lf %lf\n",gyroXrate,gyroYrate,gyroZrate);
//  printf("%lf %lf %lf\n",magX,magY,magZ);
  //#endif
  
  //#if 0 // Set to 1 to print the temperature
  //Serial.print("\t");
  //
  //double temperature = (double)tempRaw / 340.0 + 36.53;
  //Serial.print(temperature); Serial.print("\t");
  //#endif
//  delay(10);
}
 
void State_Update(void)
{
  /* Set Kalman and gyro starting angle */
  MPU6050_Update();
  MAG3110_Update();
  updatePitchRoll();
  updateYaw();
  
  setAngle(&kalmanX,roll); // First set roll starting angle
  gyroXangle = roll;
  compAngleX = roll;
  
  setAngle(&kalmanY,pitch); // Then pitch
  gyroYangle = pitch;
  compAngleY = pitch;
  
  setAngle(&kalmanZ,yaw); // And finally yaw
  gyroZangle = yaw;
  compAngleZ = yaw;
}

//****************************************
//根据加速计刷新Pitch和Roll数据
//这里采用两种方法计算roll和pitch，如果最上面没有#define RESTRICT_PITCH就采用第二种计算方法
//****************************************
void updatePitchRoll(void)
{
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll = atan2(accY,accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
  roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif
}
//****************************************
//根据磁力计刷新Yaw角
//****************************************
void updateYaw(void)
{
  double rollAngle,pitchAngle,Bfy,Bfx;  
  
  magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
  magZ *= -1;
  
  magX *= magGain[0];
  magY *= magGain[1];
  magZ *= magGain[2];
  
  magX -= magOffset[0];
  magY -= magOffset[1];
  magZ -= magOffset[2];
  
  
  rollAngle  = kalAngleX * DEG_TO_RAD;
  pitchAngle = kalAngleY * DEG_TO_RAD;
  
  Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
  Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
  yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;
  
  yaw *= -1;
}
void PID_Update(void)
{
}

void print_int(int num, int mode, int flag)  
{  
  if(num == 0){  
  if(flag == 0){  
  UsartPut('0');  
  return;  
  }else{  
  return;  
  }  
  }  
  
  print_int(num/mode, mode, 1);  
  UsartPut(sign[ num%mode ]);  
}  
  
void print_str(char const *str)  
{  
  if(str == NULL)  
  {  
  return;  
  }  
  
  while(*str != '\0')  
  {  
  UsartPut(*str);  
  str++;  
  }  
}  
  
  
void print_float(float num)  
{  
  int part = (int)num/1;  
  
  print_int(part, 10, 0);  
  UsartPut('.');  
  part=num*1000000-part*1000000;  
  print_int(part, 10, 0);  
}  
 
// 向串口发送一个字节的数据，配合printf使用
uint8_t UsartPut(uint8_t ch)
{
	USART_SendData(USART2, (uint8_t) ch);
	//Loop until the end of transmission
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {
	}
  return ch;
}

// 从串口接受数据
uint8_t UsartGet(void)
{
	while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
	return (uint8_t)USART_ReceiveData(USART2);
}

 
void check_type(char type)  
{  
  switch(type)
  {  
  case 'd':
  {  
  int num = va_arg(args, int);  
  
  if(num < 0){  
  UsartPut('-');  
  num = num * (-1);  
  }  
  
  print_int(num, 10, 0);  
  break;  
  }  
  
  case 'c':
  {  
  char ch = (char)va_arg(args, int);  
  UsartPut(ch);  
  break;  
  }  
  
  case 's':
  {  
  char *str = va_arg(args, char *);  
  print_str(str);  
  break;  
  }  
  
  case 'f':
  {  
  float num = (float)va_arg(args, double);  
  
  if(num < 0)  {  
  UsartPut('-');  
  num = num * (-1);  
  }  
  
  print_float(num);  
  break;  
  }  
  
  case 'p':
  {  
  int num = va_arg(args, int);  
  UsartPut('0');  
  UsartPut('x');  
  print_int(num, 16, 0);  
  break;  
  }  
  
  default:
  {  
  UsartPut('%');  
  UsartPut(type);  
  }  
  }  
}  
  
void myprintf(const char *format, ...)  
{  
  if(format == NULL){  
  return;  
  }  
  
  va_start(args, format);  
  
  while(*format != '\0'){  
  while(*format != '%' && *format != '\0'){  
  UsartPut(*format);  
  format++;   
  }  
  
  if(*format != '\0'){  
  format++;  
  if(*format != '\0'){  
  check_type(*format);  
  format++;  
  }else{  
  UsartPut(*(format-1));  
  }  
  }  
  }  
  
  va_end(args);  
}  

float HCSR04_Get(void)
{
  float length = 0.0f;

  GPIO_WriteBit(GPIOA,GPIO_Pin_1,1);
  DelayUs(20);
  GPIO_WriteBit(GPIOA,GPIO_Pin_1,0);
  //计数器清0
  TIM2->CNT = 0;
  while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) && TIM2->CNT<1000);
  TIM2->CNT = 0;
  tim2_count = 0;
  while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) && tim2_count < 9);
  length = (tim2_count*0xffff+TIM2->CNT)/58.8;
  return length;
}

//初始化延迟函数
void DelayInit(u8 SYSCLK)
{ 
  fac_us=SYSCLK/8;		//不论是否使用ucos,fac_us都需要使用
	fac_ms=(u16)fac_us*1000;//非ucos下,代表每个ms需要的systick时钟数
}

int get_tick_count(unsigned long *count)
{
  count[0] = SysTick->CTRL;
	return 0;
}

//延时nus
//nus为要延时的us数.
void DelayUs(u32 nus)
{
	u32 temp;
	SysTick->LOAD=nus*fac_us; //时间加载
	SysTick->VAL=0x00;  //清空计数器
	SysTick->CTRL=0x01 ;  //开始倒数
	do {
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达
	SysTick->CTRL=0x00;   //关闭计数器
	SysTick->VAL =0X00;   //清空计数器
}

//延时nms
void DelayMs(u16 nms)
{
	u32 temp;
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
  SysTick->VAL=0x00;  //清空计数器
	SysTick->CTRL=0x01 ;  //开始倒数
	do {
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达
	SysTick->CTRL=0x00;   //关闭计数器
	SysTick->VAL =0x00;   //清空计数器
}
