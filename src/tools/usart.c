#include "../main.h"

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

