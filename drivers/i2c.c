#include "conf.h"

/*标志是否读出数据*/
uint8_t i2c_test=0;
/*I2C从设备*/
/*模拟I2C端口输出输入定义*/
#define SCL_H     GPIOB->BSRR = GPIO_Pin_10
#define SCL_L     GPIOB->BRR  = GPIO_Pin_10 
#define SDA_H     GPIOB->BSRR = GPIO_Pin_11
#define SDA_L     GPIOB->BRR  = GPIO_Pin_11
#define SCL_read    GPIOB->IDR  & GPIO_Pin_10
#define SDA_read    GPIOB->IDR  & GPIO_Pin_11

/*I2C的延时函数-----------------------------------------*/
void I2C_delay(void)
{
  uint8_t i=30; //这里可以优化速度  ，经测试最低到5还能写入
  while(i) 
  { 
    i--; 
  }  
}

/*I2C的等待5ms函数--------------------------------------*/
void delay5ms(void)
{
  int i=5000;  
  while(i) 
  { 
    i--; 
  }  
}

/*I2C启动函数-------------------------------------------*/
uint8_t I2C_Start(void)
{
  SDA_H;
  SCL_H;
  I2C_delay();
  if(!SDA_read)return FALSE;  //SDA线为低电平则总线忙,退出
  SDA_L;
  I2C_delay();
  if(SDA_read) return FALSE;  //SDA线为高电平则总线出错,退出
  SDA_L;
  I2C_delay();
  return TRUE;
}

/*I2C停止函数-------------------------------------------*/
void I2C_Stop(void)
{
  SCL_L;
  I2C_delay();
  SDA_L;
  I2C_delay();
  SCL_H;
  I2C_delay();
  SDA_H;
  I2C_delay();
} 

/*I2C的ACK函数------------------------------------------*/
void I2C_Ack(void)
{  
  SCL_L;
  I2C_delay();
  SDA_L;
  I2C_delay();
  SCL_H;
  I2C_delay();
  SCL_L;
  I2C_delay();
}   

/*I2C的NoACK函数----------------------------------------*/
void I2C_NoAck(void)
{  
  SCL_L;
  I2C_delay();
  SDA_H;
  I2C_delay();
  SCL_H;
  I2C_delay();
  SCL_L;
  I2C_delay();
} 

/*I2C等待ACK函数----------------------------------------*/
uint8_t I2C_WaitAck(void)    //返回为:=1有ACK,=0无ACK
{
  SCL_L;
  I2C_delay();
  SDA_H;      
  I2C_delay();
  SCL_H;
  I2C_delay();
  if(SDA_read)
  {
    SCL_L;
    I2C_delay();
    return FALSE;
  }
  SCL_L;
  I2C_delay();
  return TRUE;
}

/*I2C发送一个uint8_t数据函数---------------------------------*/
void I2C_SendByte(uint8_t SendByte) //数据从高位到低位//
{
  uint8_t i=8;
  while(i--)
  {
    SCL_L;
    I2C_delay();
    if(SendByte&0x80)
      SDA_H;  
    else 
      SDA_L;   
    SendByte<<=1;
    I2C_delay();
    SCL_H;
    I2C_delay();
  }
  SCL_L;
}  

/*I2C读取一个uint8_t数据函数---------------------------------*/
uint8_t I2C_ReadByte(void)  //数据从高位到低位//
{ 
  uint8_t i=8;
  uint8_t ReceiveByte=0;
  
  SDA_H;        
  while(i--)
  {
    ReceiveByte<<=1;    
    SCL_L;
    I2C_delay();
    SCL_H;
    I2C_delay();  
    if(SDA_read)
    {
      ReceiveByte|=0x01;
    }
  }
  SCL_L;
  return ReceiveByte;
}  

/*I2C向指定设备指定地址写入uint8_t数据-----------------------*/
void I2C_Write(uint8_t SlaveAddress, uint8_t REG_Address,uint8_t REG_data)//单字节写入
{
  if(!I2C_Start())return;
  I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
  if(!I2C_WaitAck()){I2C_Stop(); return;}
  I2C_SendByte(REG_Address );   //设置低起始地址    
  I2C_WaitAck();  
  I2C_SendByte(REG_data);
  I2C_WaitAck();   
  I2C_Stop(); 
  delay5ms();
}

uint8_t I2C_ReadOneByte(uint8_t I2C_Addr,uint8_t addr)
{
	uint8_t res=0;
	
	I2C_Start();	
	I2C_SendByte(I2C_Addr);	   //发送写命令
	res++;
	I2C_WaitAck();
	I2C_SendByte(addr); res++;  //发送地址
	I2C_WaitAck();	  
	//I2C_Stop();//产生一个停止条件	
	I2C_Start();
	I2C_SendByte(I2C_Addr+1); res++;          //进入接收模式			   
	I2C_WaitAck();
	res=I2C_ReadByte();	   
  I2C_NoAck();
  I2C_Stop();//产生一个停止条件

	return res;
}
/*I2C向指定设备指定地址读出uint8_t数据-----------------------*/
uint8_t I2C_Read(uint8_t SlaveAddress, uint8_t REG_Address)//读取单字节
{   
  uint8_t REG_data;     
  if(!I2C_Start())return FALSE;
  I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
  if(!I2C_WaitAck()){I2C_Stop();i2c_test=1; return FALSE;}
  I2C_SendByte((uint8_t) REG_Address);   //设置低起始地址    
  I2C_WaitAck();
  I2C_Start();
  I2C_SendByte(SlaveAddress+1);
  I2C_WaitAck();
  
  REG_data= I2C_ReadByte();
  I2C_NoAck();
  I2C_Stop();
  //return TRUE;
  return REG_data;
}

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
		int i;
  if (!I2C_Start())
    return 1;
  I2C_SendByte(addr << 1 );
  if (!I2C_WaitAck()) {
    I2C_Stop();
    return 1;
  }
  I2C_SendByte(reg);
  I2C_WaitAck();
		for (i = 0; i < len; i++) {
    I2C_SendByte(data[i]);
    if (!I2C_WaitAck()) {
      I2C_Stop();
      return 0;
    }
  }
  I2C_Stop();
  return 0;
}

int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
  if (!I2C_Start())
    return 1;
  I2C_SendByte(addr << 1);
  if (!I2C_WaitAck()) {
    I2C_Stop();
    return 1;
  }
  I2C_SendByte(reg);
  I2C_WaitAck();
  I2C_Start();
  I2C_SendByte((addr << 1)+1);
  I2C_WaitAck();
  while (len) {
    if (len == 1){
      *buf = I2C_ReadByte();
      I2C_NoAck();
    }else{
      *buf = I2C_ReadByte();
      I2C_Ack();
    }
    buf++;
    len--;
  }
  I2C_Stop();
  return 0;
}
uint8_t I2CreadByte(uint8_t dev, uint8_t reg, uint8_t *data){
	*data=I2C_ReadOneByte(dev, reg);
  return 1;
}

uint8_t I2CwriteByte(uint8_t dev, uint8_t reg, uint8_t data){
    return i2cWrite(dev, reg, 1, &data);
}
uint8_t I2CwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
{

    uint8_t b;
    if (I2CreadByte(dev, reg, &b) != 0) {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return I2CwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

uint8_t I2CwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    I2CreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return I2CwriteByte(dev, reg, b);
}
