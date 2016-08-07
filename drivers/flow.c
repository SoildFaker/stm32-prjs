#include "flow.h"
#include "conf.h"
#include "tools.h"
#include "srom.h"
#include "stm32f10x_spi.h"

int X = 0,Y = 0;
int dx = 0,dy = 0;

void ADNS3080_reset(void)  //ADNS3080 复位（高）
{
  GPIO_ResetBits(GPIOB,GPIO_Pin_1); 
  delay_ms(5);
  GPIO_SetBits(GPIOB,GPIO_Pin_1);
  delay_ms(5);
  GPIO_ResetBits(GPIOB,GPIO_Pin_1);//脉冲信号
}

void Write_srom(void)
{
  int i;
  ON_CS(); 
  writr_register(0x20,0x44);
  delay_us(51);
  writr_register(0x23,0x07);
  delay_us(51);
  writr_register(0x24,0x88);
  delay_us(51);
  OFF_CS();  //突发_写模式
  delay_us(340);//等待大于1帧时间
  ON_CS(); 
  writr_register(SROM_Enable,0x18);
  OFF_CS();  //突发_写模式
  delay_us(41);//  >40us
  ON_CS();
  for(i=0;i<=1985;i++)
  {
    writr_register(0x60,SROM[i]);
    delay_us(11);// >10us
  }
  OFF_CS();
  delay_us(105); //>104us
}

void ADNS3080_Init(void)
{   
  ADNS3080_reset(); //复位
  GPIO_SetBits(GPIOB,GPIO_Pin_0);  //拉高NPD,免睡眠
  delay_ms(10);
  Write_srom();
  ADNS_Configuration();
}

void ADNS_Configuration(void)
{
  ON_CS(); 
  writr_register(Configuration_bits,0x10);   //设置分辨率 1600  //若Bit 4为0，则为400点每英寸
  delay_ms(3);
  writr_register(Extended_Config,0x01);
  delay_ms(3);
  if(read_busy()!=1) {//设为3000帧每秒
    OFF_CS();  //突发_写模式
    delay_ms(2);
    ON_CS();  
    SPI_SendReceive(Frame_Period_Max_Bound_Lower+0x80); //设置帧率 //先写低位再写高位
    SPI_SendReceive(0x40); //   C0 5000帧率    
    SPI_SendReceive(Frame_Period_Max_Bound_Upper+0x80);
    SPI_SendReceive(0x1f);   // 12
  } 
  clear_motion();
  OFF_CS();
}

uint8_t read_register(uint8_t adress)
{
  uint8_t temp;
  ON_CS();
  temp=SPI_SendReceive(adress+0x00);  //读
  delay_us(75);
  temp=SPI_SendReceive(0xff); //提供时钟信号_读
  OFF_CS();
  return temp;
}

void writr_register(uint8_t adress,uint8_t vlue)
{
  ON_CS();
  SPI_SendReceive(adress+0x80);
  SPI_SendReceive(vlue);
  OFF_CS();
  delay_us(51);
}

uint8_t read_busy(void)//写帧率的判忙  ==1忙
{
  uint8_t temp;
  ON_CS();
  temp=SPI_SendReceive(Extended_Config+0x00);
  delay_us(75);
  temp=SPI_SendReceive(0xff);
  temp&=0x80;
  OFF_CS();
  return temp;
}

void clear_motion(void)
{
  ON_CS();
  SPI_SendReceive(Motion_Clear+0x80);
  SPI_SendReceive(0xff); //清除X Y数据
  OFF_CS();
}


uint16_t read_fraps(void) //读帧率
{  
  uint16_t Frame_Period_Max_Bound_Lower1,Frame_Period_Max_Bound_Upper1;
  ON_CS();
  Frame_Period_Max_Bound_Upper1=SPI_SendReceive(Frame_Period_Uppe+0x00);
  Frame_Period_Max_Bound_Upper1=SPI_SendReceive(0xff);//接收高位的帧率
  delay_ms(5);
  Frame_Period_Max_Bound_Lower1=SPI_SendReceive(Frame_Period_Lower+0x00);
  Frame_Period_Max_Bound_Lower1=SPI_SendReceive(0xff); //接收低位的帧率
  OFF_CS();
  return ((Frame_Period_Max_Bound_Upper1 << 8) | Frame_Period_Max_Bound_Lower1);
}

void ADNS3080_Read(void)
{
  unsigned char move=0;
  int sum_x = 0,sum_y = 0;
  //burst读。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。
  ON_CS();
  SPI_SendReceive(0x50);   
  delay_us(75);
  move=SPI_SendReceive(0xFF);             
  dx=SPI_SendReceive(0xFF);
  dy=SPI_SendReceive(0xFF);
  if(dx&0x80) {
    //x的二补码转换 
    dx -= 1;
    dx = ~dx; 
    dx=(-1)*dx;
    dx-=256;
  }
  if(dy&0x80) {
    //y的二补码转换 
    dy -= 1;
    dy = ~dy; 
    dy=(-1)*dy;
    dy-=256;
  } 
  X=X+dx;             //累加X读入的移动数据
  Y=Y+dy;       //累加Y读入的移动数据
  OFF_CS();
  delay_us(4);
  OFF_CS();
  sum_x=(25.4*(float)X *H)/(12*1600);//距离=d_x*(25.4/1600)*n   其中n=像高:物高=8毫米:物长
  sum_y=(25.4*(float)Y *H)/(12*1600);
  if(move&0x10==1 || !move&0x80) {
    dx=0;
    dy=0;
  }
}

uint8_t SPI_SendReceive(uint8_t data)     //SPI1的收发
{

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);  //while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 
  SPI_I2S_SendData(SPI1, data); 
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  return SPI_I2S_ReceiveData(SPI1);
} 
