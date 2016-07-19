#ifndef __MAG_H
#define __MAG_H value

#include "conf.h"
//定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
#define    MAG3110_Addr   0x1C    //磁场传感器器件地址

extern uint8_t BUF[8];                         //接收数据缓存区                   

extern int magX, magY, magZ;    //hmc最原始数据
//**************************************
void MAG3110_Init();
void MAG3110_Update();
#endif /* ifndef __MAG_H */
