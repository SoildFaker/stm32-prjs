#ifndef _IIC_H
#define _IIC_H

#include "conf.h"
#define SCL_H         GPIOB->BSRR = GPIO_Pin_10
#define SCL_L         GPIOB->BRR  = GPIO_Pin_10 
#define SDA_H         GPIOB->BSRR = GPIO_Pin_11
#define SDA_L         GPIOB->BRR  = GPIO_Pin_11
#define SCL_read      GPIOB->IDR  & GPIO_Pin_10
#define SDA_read      GPIOB->IDR  & GPIO_Pin_11


void I2C_delay(void);
void delay5ms(void);
uint8_t I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
uint8_t I2C_WaitAck(void);
void I2C_SendByte(uint8_t SendByte);
unsigned char I2C_RadeByte(void);
void Single_WriteI2C(unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_ReadI2C(unsigned char REG_Address);
#endif /* ifndef _IIC_H */
