#ifndef __I2C
#define __I2C value

void I2C_Ack(void);
void I2C_GPIO_Config(void);
void I2C_NoAck(void);
uint8_t I2C_ReadByte(void);  //数据从高位到低位//
void I2C_SendByte(uint8_t SendByte); //数据从高位到低位//
uint8_t I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_WaitAck(void);    //返回为:=1有ACK,=0无ACKA
void I2C_delay(void);
uint8_t I2C_Read(uint8_t SlaveAddress, uint8_t REG_Address);//读取单字节
uint8_t I2C_ReadOneByte(uint8_t I2C_Addr,uint8_t addr);
uint8_t I2CreadByte(uint8_t dev, uint8_t reg, uint8_t *data);
uint8_t I2CwriteByte(uint8_t dev, uint8_t reg, uint8_t data);
uint8_t I2CwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data);
uint8_t I2CwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data);
void I2C_Write(uint8_t SlaveAddress, uint8_t REG_Address,uint8_t REG_data);//单字节写入
void delay5ms(void);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
#endif /* ifndef SYMBOL */
