#include "conf.h"

ErrorStatus HSEStartUpStatus;
// 串口相关配置
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
  USART_Init(USART2, &USART_InitStructure);
  //启动串口
  USART_Cmd(USART2, ENABLE);
}

// I2C通讯的配置，主要用来和陀螺仪芯片通讯
void I2C_Conf(void)
{
  I2C_InitTypeDef  I2C_InitStructure; 

  /*[> I2C configuration <]*/
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
  
  /*[> Apply I2C configuration after enabling it <]*/
  I2C_Init(MPU_I2Cx, &I2C_InitStructure);
  /*[> I2C Peripheral Enable <]*/
  I2C_Cmd(MPU_I2Cx, ENABLE);
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
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  //Enable GPIO timer
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  /*RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能I2C的IO口    */

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);//使能I2C的IO口    
  /*RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);//使能I2C的IO口    */

  //Enable serial timer
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //配置RCC，使能TIM2

  //开启AFIO时钟
}

// 定时器设置 
void TIMER_Conf(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
  TIM_TimeBaseStructure.TIM_Prescaler = 71;  //时钟预分频数 例如:时钟频率=72/(时钟预分频+1)  
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; ////定时器模式 向上计数  
  TIM_TimeBaseStructure.TIM_Period = 0xffff;//自动重装载寄存器周期的值(定时时间)累计 0xFFFF个频率后产生个更新或者中断(也是说定时时间到)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; ////时间分割值  
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //初始化定时器2
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //打开中断 溢出中断  

}



void GPIO_Conf(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  //USARTx_Tx为复合推挽输出
  GPIO_InitStructure.GPIO_Pin = GPIO_TxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //USARTx_Rx 为浮空输入
  GPIO_InitStructure.GPIO_Pin = GPIO_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //HC-SR04 
  GPIO_InitStructure.GPIO_Pin = TRIG_Pin; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ECHO_Pin; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //I2C配置
  /*GPIO_InitStructure.GPIO_Pin = SDA_Pin | SCL_Pin;*/
  /*GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;// 开漏输出*/
  /*GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化结构体配置*/
}

// 系统中断控制
void NVIC_Conf(void)
{ 
  NVIC_InitTypeDef NVIC_InitStructure;
  // 设置中断向量表位置 FLASH:0x08000000
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); 

  // 打开TIM2中断
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


