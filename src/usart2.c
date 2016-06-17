#include "main.h"

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


uint8_t UsartPut(uint8_t ch)
{
	USART_SendData(USARTx, (uint8_t) ch);
	//Loop until the end of transmission
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET) {
	}
  return ch;
}

uint8_t UsartGet(void)
{
	while (USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET);
	return (uint8_t)USART_ReceiveData(USARTx);
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


