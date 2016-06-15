/**
 * STM32F103C8 USRAT Demonstration
 */
#include "main.h"

/*************************** 宏定义***********************************************/
#define  USARTx                   USART2
#define  GPIOx                    GPIOA
#define  RCC_APB2Periph_GPIOx     RCC_APB2Periph_GPIOA
#define  GPIO_RxPin               GPIO_Pin_3
#define  GPIO_TxPin               GPIO_Pin_2

/******************************** 变量定义 ------------------------------------*/
ErrorStatus HSEStartUpStatus;

extern u32 counter;

/*********************************声明函数 ------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART_Configuration(void);
void TIM_Configuration(void);
u8 Uartx_PutChar(u8 ch);
void delay_us(u32 nus);
void delay_ms(u32 nms);
int usart_send_num(uint32_t num);
int msr_dist(void);
int printf(char *fmt, ...);


int main(void)
{
  RCC_Configuration();
  NVIC_Configuration();
  GPIO_Configuration();
  USART_Configuration();  
  TIM_Configuration();
  int k =0;
  long i = 0;
    
  while (1) {  
    printf("Num:%d.\r\n",i);
    k = msr_dist();
    i++;
    delay_ms(0xfffff);
  }
}

int msr_dist(void)
{
  int length;
  
  printf("Send sound.\r\n",1);
  GPIO_WriteBit(GPIOA,GPIO_Pin_1,1);
  delay_us(20);
  GPIO_WriteBit(GPIOA,GPIO_Pin_1,0);
  //计数器清0
  counter = 0;
  while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0));
  TIM_Cmd(TIM2, ENABLE);// TIM2 enable counter [允许tim2计数]
  while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)){
    if(counter>90000){
      printf(">>>ERROR:timeout_echo.\r\n",1);
      delay_ms(0x2fffff);
      break;
    }
  }
  TIM_Cmd(TIM2, DISABLE);
  
  length=counter/26.3;
  printf("trick: %d \r\nDistance:%d cm\r\n-----------------------\r\n",counter,length);
  return length;
}

void delay_ms(u32 nms)
{
  while(--nms);
}


void delay_us(u32 nus)
{
  counter = 0;
  TIM_Cmd(TIM2, ENABLE);// TIM2 enable counter
  while(nus>counter){
  }
  TIM_Cmd(TIM2, DISABLE);

}

//以指定的宽度发送十进制数的ascii码。
int usart_send_num(uint32_t num)
{
  int data[5]={0};
  int tmp = num;
  int r = 0, i;
  while(tmp>0){
    data[r] = tmp % 10;
    r++;
    tmp /= 10;
  }
  for (i = r-1; i >= 0; --i) {
    Uartx_PutChar(data[i]+'0');
  }
  if(r==0){
    Uartx_PutChar('0');
  }
  return 0;
}

int printf(char *fmt, ...)
{
   char *p = (char *)&fmt + sizeof(fmt);
   uint8_t temp = 0;
   while (1) {
    temp = *fmt;
    if (temp == '\0')
      break;
    /*else if (temp == '\\')*/
      /*Uartx_PutChar(*++fmt);*/
    else if (temp == '%') {
      fmt++;
      if (*fmt == 'd')
        usart_send_num(*((int *)p));
      p += sizeof(int);
    }else{
      Uartx_PutChar(temp);
    }
    fmt++;
  }
   return 0;
}


u8 Uartx_PutChar(u8 ch)
{
  /* Write a character to the USART */
  USART_SendData(USARTx, (u8) ch);
  while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET) {
  }
  return ch;
}


void USART_Configuration(void)
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

void RCC_Configuration(void)
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

void TIM_Configuration(void)
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
/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description  : Configures the different GPIO ports.
* Input      : None
* Output     : None
* Return     : None
*******************************************************************************/
void GPIO_Configuration(void)
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

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description  : Configures Vector Table base location.
* Input      : None
* Output     : None
* Return     : None
*******************************************************************************/
void NVIC_Configuration(void)
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


#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description  : Reports the name of the source file and the source line number
*          where the assert_param error has occurred.
* Input      : - file: pointer to the source file name
*          - line: assert_param error line source number
* Output     : None
* Return     : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
