/*
* STM32F1_USART1.c
*
*  Created on: 2013-11-13
*      Author: Administrator
*/



#include "../src/conf.h"

#define USE_REMAP		0//串口1重映射 0:非重映射;1:重映射
#define RX_BUF_SIZE  0x100
#define PRINTF_BUF_SIZE  0x200


static uint8_t USART1_RxBuffer[RX_BUF_SIZE];//串口1内部接收缓存
//static uint8_t USART1_TxBuffer[0x100];
static uint8_t print_buffer[PRINTF_BUF_SIZE];//打印缓存

static volatile uint32_t USART1_ReadIndex = 0;//读索引
static volatile uint8_t sendDoneFlag = 1;//发送完成标志
static volatile uint8_t recvDoneFlag = 0;//接收完成标志

/**
* 串口初始化
* @param baud 波特率
*/
void USART1_Configuration(uint32_t baud)
{
    
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_USART1|
                               RCC_APB2Periph_AFIO,ENABLE);
    
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#if USE_REMAP
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);
#else
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
    
    
	DMA_InitTypeDef DMA_InitStructure;
    
	DMA_DeInit(DMA1_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&USART1_RxBuffer[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = sizeof(USART1_RxBuffer);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(DMA1_Channel5, ENABLE);
    
    
	DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
	//	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&USART1_TxBuffer[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	//	DMA_InitStructure.DMA_BufferSize = sizeof(USART1_TxBuffer);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
	USART_Cmd(USART1, ENABLE);
}


/**
* 串口发送函数
* @param buffer	发送内容
* @param length	发送长度
* @return
*/
uint16_t USART1_SendBuffer(const uint8_t* buffer, uint16_t length)
{
	if( (buffer==0) || (length==0) )
	{
		return 0;
	}
	sendDoneFlag = 0;
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA1_Channel4->CMAR = (uint32_t)buffer;
	DMA_SetCurrDataCounter(DMA1_Channel4, length);
	DMA_Cmd(DMA1_Channel4, ENABLE);
	return length;
}

/**
* 发送完成标志查询
* @return	1:发送完成 	0:未完成
*/
uint8_t isUSART1SendDone(void)
{
	return sendDoneFlag;
}


/**
*Printf打印函数
* @param format	Printf格式
*/
void Debug_Printf(const char *format, ...)
{
	uint32_t length;
	va_list args;
	va_start(args, format);
	length = vsnprintf((char*)print_buffer, sizeof(print_buffer), (char*)format, args);//格式化内容
	va_end(args);
	USART1_SendBuffer(print_buffer,length);//发送
	while(!sendDoneFlag);//等待发送完成
}


/**
* 打印16进制数组
* @param hex			数组
* @param hex_length	数组长度
*/
void Printf_Hex(const uint8_t* hex, uint16_t hex_length)
{
	const uint8_t char_table[] = "0123456789ABCDEF";
	uint16_t j=0;
	for(uint16_t i=0;(i<hex_length)&&j<sizeof(print_buffer);i++)
	{
		print_buffer[j++] = char_table[(hex[i]&0xF0)>>4];
		print_buffer[j++] = char_table[hex[i]&0x0F];
		print_buffer[j++] = ' ';
	}
	print_buffer[j++] = '\n';
	USART1_SendBuffer(print_buffer,j);//发送
	while(!sendDoneFlag);//等待发送完成
}

/**
* 接收数据剩余量查询
* @return	 剩余量
*/
uint32_t USART1_DataAvailable(void)
{
    uint32_t remain_length;
    uint32_t write_index;
    write_index = sizeof(USART1_RxBuffer) - DMA_GetCurrDataCounter(DMA1_Channel5);
    
    if( USART1_ReadIndex > write_index )
    {
        remain_length = (sizeof(USART1_RxBuffer) - USART1_ReadIndex) + write_index;
    }
    else
    {
        remain_length = write_index - USART1_ReadIndex;
    }
    return remain_length;
    
}


/**
* 串口接收函数
* @param buffer		接收缓存
* @param max_length	接收缓存的大小,若max_length小于数据的长度,则接收完成标志不被清0
* @return 接收到数据的长度
*/
uint32_t USART1_RecvBuffer(uint8_t* buffer, uint32_t max_length)
{
    uint32_t recv_length;
    uint32_t write_idx;
    if((buffer==0) || (max_length==0))
    {
        return 0;
    }
    write_idx = sizeof(USART1_RxBuffer) - DMA_GetCurrDataCounter(DMA1_Channel5);
    if(USART1_ReadIndex<write_idx)//计算接收到的长度
    {
        recv_length = write_idx - USART1_ReadIndex;
    }
    else
    {
        recv_length = sizeof(USART1_RxBuffer) - USART1_ReadIndex + write_idx;
    }
    for(uint16_t i=0;(i<recv_length) && (i<max_length);i++)//提取数据内容
    {
        if( (USART1_ReadIndex+i) == sizeof(USART1_RxBuffer))
        {
            USART1_ReadIndex = 0;
        }
        buffer[i] = USART1_RxBuffer[USART1_ReadIndex];
        USART1_ReadIndex++;
    }
    if(USART1_DataAvailable()==0)//是否从RxBuffer读取完数据?
    {
        recvDoneFlag = 0;//数据已读完,清接收完成标志
    }
    return recv_length;
}


/**
* 接收完成标志查询
* @return	1:接收完成	0:未完成
*/
uint8_t isUSART1RecvDone(void)
{
	return recvDoneFlag;
}

/**
* 清空串口接收缓存
* @return	串口接收缓存残留数据的长度
*/
uint32_t USART1_Flush(void)
{
	uint32_t flush_length;
	uint32_t write_index;
    
	write_index = sizeof(USART1_RxBuffer) - DMA_GetCurrDataCounter(DMA1_Channel5);
    
	if( USART1_ReadIndex > write_index )
	{
		flush_length = (sizeof(USART1_RxBuffer) - USART1_ReadIndex) + write_index;
	}
	else
	{
		flush_length = write_index - USART1_ReadIndex;
	}
    
	USART1_ReadIndex = write_index;
    recvDoneFlag = 0;//清接收完成标志
	return flush_length;
}


/**
* 串口中断
*/
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_IDLE);//空闲
		USART_ReceiveData(USART1);//空读清状态
		recvDoneFlag = 1;
	}
}


/**
* 串口1DMA发送完成中断
*/
void DMA1_Channel4_IRQHandler(void)
{
	if ( DMA_GetITStatus(DMA1_IT_TC4) )
	{
		DMA_ClearITPendingBit(DMA1_IT_TC4);
		sendDoneFlag = 1;
	}
}
