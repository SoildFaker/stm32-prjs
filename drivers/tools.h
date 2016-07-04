#ifndef __TOOLS
#define __TOOLS value

#include "conf.h"

int HCSR04_Get(void);
uint8_t UsartPut(uint8_t ch);
void getAttitude(int16_t* AccelGyro);
uint8_t UsartGet(void);
void DelayInit(u8 SYSCLK);
void DelayUs(u32 nus);
void DelayMs(u16 nms);
void UserInit(void);
uint8_t I2C_Write(uint8_t HardwareAddr, uint8_t WriteAddr, uint8_t NumByteToWrite, uint8_t *pData);
void I2C_ByteWrite(uint8_t HardwareAddr, uint8_t WriteAddr, uint8_t data);
uint8_t I2C_Read(uint8_t HardwareAddr, uint8_t ReadAddr, uint16_t NumByteToRead, uint8_t *pBuffer);
uint8_t I2C_ByteRead(uint8_t HardwareAddr, uint8_t ReadAddr);
void I2C_BitsWrite(uint8_t HardwareAddr, uint8_t WriteAddr, uint8_t BitStart, uint8_t Length, uint8_t Data);
void I2C_BitWrite(uint8_t HardwareAddr, uint8_t WriteAddr, uint8_t BitNum, uint8_t Data);
#endif /* ifndef __TOOLS */
