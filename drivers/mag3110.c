#include "mag3110.h"
#include "i2c.h"
#include "tools.h"

int magX, magY, magZ;
uint8_t BUF[8];                         //接收数据缓存区                   
//**************************************
//初始化MAG3110，根据需要请参考pdf进行修改
//**************************************
void MAG3110_Init()
{
  Single_WriteI2C(MAG3110_Addr, 0x11, 0x80);
  Single_WriteI2C(MAG3110_Addr, 0x10,0x12);
}
//**************************************
//从MAG3110连续读取6个数据放在BUF中
//**************************************
void MAG3110_Update()
{
  BUF[1]=Single_ReadI2C(MAG3110_Addr, 0x02);//OUT_X_L_A
  BUF[2]=Single_ReadI2C(MAG3110_Addr, 0x01);//OUT_X_H_A
  BUF[3]=Single_ReadI2C(MAG3110_Addr, 0x04);//OUT_Y_L_A
  BUF[4]=Single_ReadI2C(MAG3110_Addr, 0x03);//OUT_Y_H_A
  BUF[5]=Single_ReadI2C(MAG3110_Addr, 0x06);//OUT_Z_L_A
  BUF[6]=Single_ReadI2C(MAG3110_Addr, 0x05);//OUT_Y_H_A
  
  magX=(BUF[1] << 8) | BUF[2]; //Combine MSB and LSB of X Data output register
  magY=(BUF[3] << 8) | BUF[4]; //Combine MSB and LSB of Y Data output register
  magZ=(BUF[5] << 8) | BUF[6]; //Combine MSB and LSB of Z Data output register
  myprintf("magX:%d\tmagY:%d\tmagZ:%d\r\n", magX, magY, magZ);
  Single_WriteI2C(MAG3110_Addr, 0x10,0x12);

}
