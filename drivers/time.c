#include "time.h"

static u8  fac_us=0;//us延时倍乘数			   
static u16 fac_ms=0;//ms延时倍乘数
ErrorStatus HSEStartUpStatus;

//初始化延迟函数
void delay_init(u8 SYSCLK)
{
	fac_us=SYSCLK/8;		//不论是否使用ucos,fac_us都需要使用
	fac_ms=(u16)fac_us*1000;//非ucos下,代表每个ms需要的systick时钟数   
}								  

//延时nus
//nus为要延时的us数.		  								   
void delay_us(u32 nus)
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
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;       //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数  
	do {
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;     //关闭计数器
	SysTick->VAL =0X00;     //清空计数器	  	  
} 


void RCC_Conf(void)
{
  //复位RCC外部设备寄存器到默认值
  RCC_DeInit();  
  //打开外部高速晶振
  RCC_HSEConfig(RCC_HSE_ON); 
  //等待外部高速时钟准备好
  HSEStartUpStatus = RCC_WaitForHSEStartUp(); 
  if(HSEStartUpStatus == SUCCESS){
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
  FLASH_SetLatency(FLASH_Latency_2);
  //配置AHB(HCLK)时钟=SYSCLK
  RCC_HCLKConfig(RCC_SYSCLK_Div1);
  //配置APB2(PCLK2)钟=AHB时钟
  RCC_PCLK2Config(RCC_HCLK_Div1); 
  //配置APB1(PCLK1)钟=AHB 1/2时钟
  RCC_PCLK1Config(RCC_HCLK_Div2);  
  //配置ADC时钟=PCLK2 1/4
  RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
  //配置PLL时钟 == 外部高速晶体时钟*9
  RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); 
  //配置ADC时钟= PCLK2/4
  RCC_ADCCLKConfig(RCC_PCLK2_Div4);
  //使能PLL时钟
  RCC_PLLCmd(ENABLE);  
  //等待PLL时钟就绪
  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)  {
  }
  //配置系统时钟 = PLL时钟
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 
  //检查PLL时钟是否作为系统时钟
  while(RCC_GetSYSCLKSource() != 0x08)  {
  }
  }
  
  //Enable GPIO timer
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  //Enable serial timer
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

}

// 定时器设置 
void TIMER_Conf(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //配置RCC，使能TIM2
  TIM_TimeBaseStructure.TIM_Prescaler = 71;  //时钟预分频数 例如:时钟频率=72/(时钟预分频+1)  
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; ////定时器模式 向上计数  
  TIM_TimeBaseStructure.TIM_Period = 0xffff;//自动重装载寄存器周期的值(定时时间)累计 0xFFFF个频率后产生个更新或者中断(也是说定时时间到)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; ////时间分割值  
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //初始化定时器2
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //打开中断 溢出中断  

}


