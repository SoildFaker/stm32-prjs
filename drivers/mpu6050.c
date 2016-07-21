#include "mpu6050.h"
#include "tools.h"
#include <math.h>
//---------------------------------------------------
int16_t gyro_x,gyro_y,gyro_z;
int16_t acc_x,acc_y,acc_z;
float gyro_x_rate,gyro_y_rate,gyro_z_rate;
float acc_x_temp,acc_y_temp,acc_z_temp;
uint8_t test,i2c_sdata[20],i2c_rdata[20];

void MPU6050_WriteBlock(uint8_t adr, uint8_t data[], uint8_t data_len)//okie
{
  uint8_t i;
  // Send START condition 
  I2C_GenerateSTART(MPU_I2Cx, ENABLE);
  //loop until match the 'start condition' flag
  while(!I2C_GetFlagStatus(MPU_I2Cx, I2C_FLAG_SB));
  
  // send mpu6050 addres and check until it's done
  I2C_Send7bitAddress(MPU_I2Cx, MPU6050_WADR, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
  // send mpu6050 the begining address where we want to send data, check until it's done
  I2C_SendData(MPU_I2Cx,adr);
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  //then send continous data in data[] array
  for(i=0;i<data_len;i++)
  {
    I2C_SendData(MPU_I2Cx,data[i]);
    while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    delay_ms(1);
  }
  
  // send stop condition and wait until done 
  I2C_GenerateSTOP(MPU_I2Cx, ENABLE);
  while(I2C_GetFlagStatus(MPU_I2Cx, I2C_FLAG_STOPF));
}

void MPU6050_ReadBlock(uint8_t adr, uint8_t data[], uint8_t data_len)
//adr: addres of the first reg what we read from
//data[]: array of data, which we store read data in
//data_len: lenght of reading data
{
  uint8_t i;
  
  while(I2C_GetFlagStatus(MPU_I2Cx, I2C_FLAG_BUSY));
  // Send START condition 
  I2C_GenerateSTART(MPU_I2Cx, ENABLE);
  //loop until match the 'start condition' flag
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
  
  // send mpu6050 7 bit address and a Write bit, check until done
  I2C_Send7bitAddress(MPU_I2Cx, MPU6050_WADR, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  //I2C_Cmd(MPU_I2Cx, ENABLE);// clear all the event  
    
  // send mpu6050 the addres of the reg we want to read data from 
  I2C_SendData(MPU_I2Cx,adr);
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  // send stop condition and wait until done 
  I2C_GenerateSTOP(MPU_I2Cx, ENABLE);
  while(I2C_GetFlagStatus(MPU_I2Cx, I2C_FLAG_STOPF));
    
  // Send START condition 
  I2C_GenerateSTART(MPU_I2Cx, ENABLE);
  //loop until match the 'start condition' flag
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
  
  
  // send mpu6050 7 bit addres and Read command
  I2C_Send7bitAddress(MPU_I2Cx, MPU6050_RADR, I2C_Direction_Receiver);
  while(!I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  
  i=0;
  while(data_len)
  {
    if(data_len==1){
      I2C_AcknowledgeConfig(MPU_I2Cx, DISABLE);
      I2C_GenerateSTOP(MPU_I2Cx, ENABLE);
    }
    if(I2C_CheckEvent(MPU_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
      i2c_rdata[i] = I2C_ReceiveData(MPU_I2Cx);
      i++;
      data_len--;
    }
  }
  I2C_AcknowledgeConfig(MPU_I2Cx, ENABLE);
}

void MPU6050_Init(void)
{
  //reset mpu6050
  i2c_sdata[0]=0x80;
  MPU6050_WriteBlock(Res107,i2c_sdata,1);
  delay_ms(10);
  
  //use internal clock
  i2c_sdata[0]=Res107_value;
  MPU6050_WriteBlock(Res107,i2c_sdata,1);
  delay_ms(10);
  
  // enable i2c master mode, arm uc will contrl the mpu6050 bypass auxilary bus
  i2c_sdata[0]=Res106_value;
  MPU6050_WriteBlock(Res106,i2c_sdata,1);
  delay_ms(10);
  
  //enable bypass i2c bus
  i2c_sdata[0]=Res55_value;
  MPU6050_WriteBlock(Res55,i2c_sdata,1);
  delay_ms(10);
  
  //config our meure mode, read the specific res for more info
  i2c_sdata[0]=Res25_value;
  i2c_sdata[1]=Res26_value;
  i2c_sdata[2]=Res27_value;
  i2c_sdata[3]=Res28_value;
  MPU6050_WriteBlock(Res25,i2c_sdata,4);
  delay_ms(10);
  //printf("mpu6050 init done\r");
  GPIOB->ODR|=1<<0;
}


void MPU6050_Read(void)
{
  
  MPU6050_ReadBlock(start_read_address,i2c_rdata,14);
  
  acc_x=(int16_t)(i2c_rdata[0]<<8|i2c_rdata[1]);
  acc_y=(int16_t)(i2c_rdata[2]<<8|i2c_rdata[3]);
  acc_z=(int16_t)(i2c_rdata[4]<<8|i2c_rdata[5]);
  /*temp=(float)((int16_t)(i2c_rdata[6]<<8|i2c_rdata[7]))/340+36.53;*/
  gyro_x=(int16_t)(i2c_rdata[8]<<8|i2c_rdata[9]);
  gyro_y=(int16_t)(i2c_rdata[10]<<8|i2c_rdata[11]);
  gyro_z=(int16_t)(i2c_rdata[12]<<8|i2c_rdata[13]);

}

void MPU_GetGyroRate(void)
{
  gyro_x_rate=(float) (gyro_x-gyro_zero_x)/gyro_sensitivity;  
  gyro_y_rate=(float) (gyro_y-gyro_zero_y)/gyro_sensitivity;  
  gyro_z_rate=(float) (gyro_z-gyro_zero_z)/gyro_sensitivity;  
}

void MPU_GetAccValue(void)
{
  acc_x_temp= (float)acc_x/acc_x_gain;
  acc_y_temp= (float)acc_y/acc_y_gain;
  acc_z_temp= (float)acc_z/acc_z_gain;// [acc_z]=g=9.8 m/s.s
}
