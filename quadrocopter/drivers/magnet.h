#ifndef __MAGNET_H
#define __MAGNET_H

#define MAG_Addr              0x1C
#define STATUS_REG            0x00		/*STATUS Registers*/
/*
**  XYZ Data Registers
*/
#define OUT_X_MSB_REG         0x01
#define OUT_X_LSB_REG         0x02
#define OUT_Y_MSB_REG         0x03
#define OUT_Y_LSB_REG         0x04
#define OUT_Z_MSB_REG         0x05
#define OUT_Z_LSB_REG         0x06  
#define WHO_AM_I_REG          0x07

#define CTRL_REG1             0x10		 /*CTRL_REG1 System Control 1 Register*/
#define CTRL_REG2             0x11	

extern uint16_t xsdat,ysdat,zsdat;

void Data_Manage(uint8_t *BUF);
void MAG3110_Init();
void I2C_ByteWrite(u8 DevAddr,u8 WData, u8 WriteAddr);
void I2C_BufferRead(u8 DevAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
#endif /* __MAGNET_H */
