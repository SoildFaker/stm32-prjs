#ifndef __FLOW_H
#define __FLOW_H value

#include "stm32f10x.h"
#define H  150 //毫米

#define  ON_CS()    {GPIO_ResetBits(GPIOA,GPIO_Pin_4);}//delay_us(1);
#define  OFF_CS()   {GPIO_SetBits(GPIOA,GPIO_Pin_4);}//delay_us(1);

#define Product_ID   0x00
#define Motion 	     0x02
#define DX		     0X03
#define DY		     0X04
#define SQUAL		 0x05
#define Pixel_Sum	 0x06
#define Configuration_bits	 0x0a
#define Extended_Config      0x0b
#define Frame_Period_Lower	 0x10
#define Frame_Period_Uppe	 0x11
#define	Motion_Clear	     0x12
#define	Frame_Capture	     0x13
#define	SROM_Enable		     0x14
#define Frame_Period_Max_Bound_Lower	0x19
#define Frame_Period_Max_Bound_Upper    0x1a
#define Frame_Period_Min_Bound_Lower    0x1b
#define Frame_Period_Min_Bound_Upper    0x1c
#define Shutter_Max_Bound_Lower         0x1d
#define Shutter_Max_Bound_Upper         0x1e
#define SROM_ID	         0x1f
#define	Pixel_Burst	         0x40
#define	Motion_Burst         0x50
#define	SROM_Load	         0x60

extern int X,Y;
extern int dx,dy;

void Write_srom(void);
void ADNS3080_Init(void);
void ADNS_Configuration(void);
void ADNS3080_reset(void);
void writr_register(u8 adress,u8 vlue);
u8 read_register(u8 adress);
u8 read_busy(void);//ÐŽÖ¡ÂÊµÄÅÐÃŠ
void clear_motion(void);
u16 read_fraps(void); //¶ÁÖ¡ÂÊ
void ADNS3080_Read(void);
uint8_t SPI_SendReceive(uint8_t data);

#endif /* ifndef __FLOW_H */
