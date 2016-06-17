#include "hcsr04.h"


int HCSR04_Get(void)
{
  int length;
  
  GPIO_WriteBit(GPIOA,GPIO_Pin_1,1);
  delay_us(20);
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

void GPIO_Conf(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  //开启AFIO时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
   //配置USARTx_Tx为复合推挽输出
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //配置 USARTx_Rx 为浮空输入
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //HC-SR02 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

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



