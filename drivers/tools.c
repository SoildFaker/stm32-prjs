#include "tools.h"

static u8  fac_us=0;//us延时倍乘数			   
static u16 fac_ms=0;//ms延时倍乘数

void UserInit(void)
{
  RCC_Conf();
  NVIC_Conf();
  GPIO_Conf();
  USART_Conf();  
  TIMER_Conf();

  I2C_Conf();
  DelayInit(72);
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

// IIC写字节
void I2C_ByteWrite(uint8_t HardwareAddr, uint8_t WriteAddr, uint8_t data)
{
  I2C_GenerateSTART(MPU_I2Cx, ENABLE);
  printf("debug:A\r\n");
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
  printf("debug:B\r\n");
  I2C_Send7bitAddress(MPU_I2Cx, HardwareAddr,I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  I2C_SendData(MPU_I2Cx, WriteAddr);
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  I2C_SendData(MPU_I2Cx, data);
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  I2C_GenerateSTOP(MPU_I2Cx, ENABLE);
}

// IIC写数据
uint8_t I2C_Write(uint8_t HardwareAddr, uint8_t WriteAddr, uint8_t NumByteToWrite, uint8_t *pData)
{
  I2C_GenerateSTART(MPU_I2Cx, ENABLE);
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(MPU_I2Cx, HardwareAddr,I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  I2C_SendData(MPU_I2Cx, WriteAddr);
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  while(NumByteToWrite){
    if(I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
      I2C_SendData(MPU_I2Cx, *pData);
      pData++;
      NumByteToWrite--;
    }
  }
  I2C_GenerateSTOP(MPU_I2Cx, ENABLE);
  return 0;
}

// I2C 读字节
uint8_t I2C_ByteRead(uint8_t HardwareAddr, uint8_t ReadAddr)
{
  uint8_t receive = 0;
  while(I2C_GetFlagStatus(MPU_I2Cx, I2C_FLAG_BUSY));//检查I2C是否忙碌中
  I2C_GenerateSTART(MPU_I2Cx, ENABLE);//产生开始信号
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_MODE_SELECT));//清除EV5
  I2C_Send7bitAddress(MPU_I2Cx, HardwareAddr, I2C_Direction_Transmitter);//写入器件
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//清除EV6
  I2C_SendData(MPU_I2Cx, ReadAddr);//发送要读取的地址
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//清除EV8
  I2C_GenerateSTART(MPU_I2Cx, ENABLE);//开启信号
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_MODE_SELECT));//清除EV5
  I2C_Send7bitAddress(MPU_I2Cx, HardwareAddr, I2C_Direction_Receiver);//读数据
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));//清除EV6
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
  receive = I2C_ReceiveData(MPU_I2Cx);
  I2C_AcknowledgeConfig(MPU_I2Cx, DISABLE);
  I2C_GenerateSTOP(MPU_I2Cx, ENABLE);
  return receive;
}


// I2C 读连续数据
uint8_t I2C_Read(uint8_t HardwareAddr, uint8_t ReadAddr, uint16_t NumByteToRead, uint8_t *pBuffer)
{
  while(I2C_GetFlagStatus(MPU_I2Cx, I2C_FLAG_BUSY));//检查I2C是否忙碌中
  I2C_GenerateSTART(MPU_I2Cx, ENABLE);//产生开始信号
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_MODE_SELECT));//清除EV5
  I2C_Send7bitAddress(MPU_I2Cx, HardwareAddr, I2C_Direction_Transmitter);//写入器件
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//清除EV6
  I2C_SendData(MPU_I2Cx, ReadAddr);//发送要读取的地址
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//清除EV8
  I2C_GenerateSTART(MPU_I2Cx, ENABLE);//开启信号
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_MODE_SELECT));//清除EV5
  I2C_Send7bitAddress(MPU_I2Cx, HardwareAddr, I2C_Direction_Receiver);//读数据
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));//清除EV6
  while(NumByteToRead){//接受数据
    if (NumByteToRead == 1) {//最后一个数据时关闭应答
      I2C_AcknowledgeConfig(MPU_I2Cx, DISABLE);
      I2C_GenerateSTOP(MPU_I2Cx, ENABLE);
    }
    if (I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
      *pBuffer = I2C_ReceiveData(MPU_I2Cx);
      pBuffer++;
      NumByteToRead--;
    }
  }
  return 0;

}
// 读修改写 指定设备 指定寄存器一个字节 中的多个位
void I2C_BitsWrite(uint8_t HardwareAddr, uint8_t WriteAddr, uint8_t BitStart, uint8_t Length, uint8_t Data)
{
  uint8_t b;
  b = I2C_ByteRead(HardwareAddr, WriteAddr);
  printf("debug:%d\r\n",b);
  u8 mask = (0xFF << (BitStart + 1)) | 0xFF >> ((8 - BitStart) + Length - 1);
  Data <<= (8 - Length);
  Data >>= (7 - BitStart);
  b &= mask;
  b |= Data;
  printf("debug:%d\r\n",b);
  I2C_ByteWrite(HardwareAddr, WriteAddr, b);

}
//读修改写 指定设备 指定寄存器一个字节 中的1个位
void I2C_BitWrite(uint8_t HardwareAddr, uint8_t WriteAddr, uint8_t BitNum, uint8_t Data)
{
  uint8_t b;
  b = I2C_ByteRead(HardwareAddr, WriteAddr);
  b = (Data != 0) ? (b | (1 << BitNum)) : (b & ~(1 << BitNum));
  I2C_ByteWrite(HardwareAddr, WriteAddr, b);
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
	SysTick->VAL =0x00;       //清空计数器
}



