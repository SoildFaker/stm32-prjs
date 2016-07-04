#include "tools.h"

static u8  fac_us=0;//us延时倍乘数			   
static u16 fac_ms=0;//ms延时倍乘数

void UserInit(void)
{
  RCC_Conf();
  GPIO_Conf();
  NVIC_Conf();
  TIMER_Conf();
  USART_Conf();  

  MPU6050_I2C_Init();
  MPU6050_Initialize();
}

int HCSR04_Get(void)
{
  int length;
  
  GPIO_WriteBit(GPIOA,GPIO_Pin_1,1);
  DelayUs(20);
  GPIO_WriteBit(GPIOA,GPIO_Pin_1,0);
  //计数器清0
  TIM2->CNT = 0;
  TIM_Cmd(TIM2, ENABLE);// TIM2 enable counter [允许tim2计数]
  while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) && TIM2->CNT<1000);
  TIM2->CNT = 0;
  while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) && TIM2->CNT<60000);
  TIM_Cmd(TIM2, DISABLE);

  length = TIM2->CNT/58.8;
  return length;
}

void getAttitude(int16_t* attitude)
{
  double gyrofac = 8.192*2;
  double accelfac = 6.536;
  int16_t AccelGyro[6];
  double x,y,z;
  MPU6050_GetRawAccelGyro(AccelGyro);
  /*AccelGyro[0]/=gyrofac; AccelGyro[1]/=gyrofac; AccelGyro[2]/=gyrofac;*/
  x=(float)AccelGyro[0]/1000;y=(float)AccelGyro[1]/1000;z=(float)AccelGyro[2]/1000;
  AccelGyro[3]/=accelfac; AccelGyro[4]/=accelfac; AccelGyro[5]/=accelfac;
  attitude[0]=atan(y/x)*1000;
  attitude[1]=atan(x/y)*1000;
  attitude[2]=atan(x/z)*1000;
}

int _read(int file, char *ptr, int len) 
{
  int n;
  int num = 0;
  for (n = 0; n < len; n++) {
     char c = UsartGet();
    *ptr++ = c;
    num++;
  }
  return num;
}

int _write(int file, char *ptr, int len) 
{
  int n;
  for (n = 0; n < len; n++) {
     UsartPut(*ptr++);
  }
  return len;
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

//初始化延迟函数
void DelayInit(u8 SYSCLK)
{
	fac_us=SYSCLK/8;		//不论是否使用ucos,fac_us都需要使用
	fac_ms=(u16)fac_us*1000;//非ucos下,代表每个ms需要的systick时钟数   
}								  

//延时nus
//nus为要延时的us数.		  								   
void DelayUs(u32 nus)
{		
	u32 temp;	  	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;    //清空计数器
	SysTick->CTRL=0x01 ;    //开始倒数 	 
	do {
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;     //关闭计数器
	SysTick->VAL =0X00;     //清空计数器	 
}

//延时nms
void DelayMs(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
  SysTick->VAL=0x00;    //清空计数器
	SysTick->CTRL=0x01 ;    //开始倒数 	 
	do {
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;     //关闭计数器
	SysTick->VAL =0x00;       //清空计数器
}



