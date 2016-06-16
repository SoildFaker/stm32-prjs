#include "lib.h"

ErrorStatus HSEStartUpStatus;
static u8  fac_us=0;//us延时倍乘数			   
static u16 fac_ms=0;//ms延时倍乘数,在ucos下,代表每个节拍的ms数
extern u16 timer_counter;
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
  USART_SendData(USARTx, (u16) ch);
  while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET) {
  }
	return ch;
}
#endif 

int HCSR04_Get(void)
{
  int length;
  
  printf("Send sound.\r\n",1);
  GPIO_WriteBit(GPIOA,GPIO_Pin_1,1);
  delay_us(20);
  GPIO_WriteBit(GPIOA,GPIO_Pin_1,0);
  //计数器清0
  timer_counter = 0;
  while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0));
  TIM_Cmd(TIM2, ENABLE);// TIM2 enable counter [允许tim2计数]
  while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) && timer_counter <10000){
  }
  TIM_Cmd(TIM2, DISABLE);
  
  length=timer_counter/58.8;
  printf("trick: %d \r\nDistance:%d cm\r\n-----------------------\r\n",timer_counter,length);
  return length;
}

void USART_Conf(void)
{
  USART_InitTypeDef USART_InitStructure;
  //串口配置： 波特率 115200 数据位 8 停止位 1  奇偶位 NONE  
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity =  USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    //初始化串口
  USART_Init(USARTx, &USART_InitStructure);
  //启动串口
  USART_Cmd(USARTx, ENABLE);
}

void NVIC_Conf(void)
{ 
  NVIC_InitTypeDef NVIC_InitStructure;
#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif
  /* Enable the TIM2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 0, 0);
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
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOx, ENABLE);
  
  //Enable serial timer
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

}

void TIMER_Conf(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //配置RCC，使能TIMx
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 35;  //时钟预分频数 例如:时钟频率=72/(时钟预分频+1)  
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; ////定时器模式 向上计数  
  TIM_TimeBaseStructure.TIM_Period = 1;//自动重装载寄存器周期的值(定时时间)累计 0xFFFF个频率后产生个更新或者中断(也是说定时时间到)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; ////时间分割值  
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //初始化定时器2
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //打开中断 溢出中断  

}



void GPIO_Conf(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  //开启AFIO时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
   //配置USARTx_Tx为复合推挽输出
  GPIO_InitStructure.GPIO_Pin = GPIO_TxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOx, &GPIO_InitStructure);
  
  //配置 USARTx_Rx 为浮空输入
  GPIO_InitStructure.GPIO_Pin = GPIO_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOx, &GPIO_InitStructure);
  
  //HC-SR02 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

}
//初始化延迟函数
//当使用ucos的时候,此函数会初始化ucos的时钟节拍
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void delay_init(u8 SYSCLK)
{
 	SysTick->CTRL&=~(1<<2);	//SYSTICK使用外部时钟源	 
	fac_us=SYSCLK/8;		//不论是否使用ucos,fac_us都需要使用
	    
	fac_ms=(u16)fac_us*1000;//非ucos下,代表每个ms需要的systick时钟数   
}								    

//延时nus
//nus为要延时的us数.		    								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL=0x01 ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  	    
} 



