#ifndef _MPU6050_H
#define _MPU6050_H

#include "conf.h"

/*
#define   ACCEL_XOUT_H  59  // R  ACCEL_XOUT[15:8] 
#define   ACCEL_XOUT_L  60  // R  ACCEL_XOUT[7:0] 
#define   ACCEL_YOUT_H  61  // R  ACCEL_YOUT[15:8] 
#define   ACCEL_YOUT_L  62  // R  ACCEL_YOUT[7:0] 
#define   ACCEL_ZOUT_H  63  // R  ACCEL_ZOUT[15:8] 
#define   ACCEL_ZOUT_L  64  // R  ACCEL_ZOUT[7:0] 
#define   TEMP_OUT_H  65  // R  TEMP_OUT[15:8] 
#define   TEMP_OUT_L  66  // R  TEMP_OUT[7:0] 
#define   GYRO_XOUT_H 67  // R  GYRO_XOUT[15:8] 
#define   GYRO_XOUT_L 68  // R  GYRO_XOUT[7:0] 
#define   GYRO_YOUT_H 69  // R  GYRO_YOUT[15:8] 
#define   GYRO_YOUT_L 70  // R  GYRO_YOUT[7:0] 
#define   GYRO_ZOUT_H 71  // R  GYRO_ZOUT[15:8] 
#define   GYRO_ZOUT_L 72  // R  GYRO_ZOUT[7:0] 
*/
//------- define data for MPU6050 setup ------------
#define MPU6050_SLA_ADR 0x68 
#define MPU6050_WADR MPU6050_SLA_ADR<<1
#define MPU6050_RADR (MPU6050_SLA_ADR<<1)|0x01
#define Res25_value 0x07  // Sample rate divider
#define Res26_value 0x00  // Configuration        
#define Res27_value 0x00  // Gyroscope configuration: gyro_scale= +-250 degree/s; sen=131 LSB/degree/s
#define Res28_value 0x00  // Accelerometter configuration  : accelero_scale=+-2g ; sen=16384 LSB/g       
#define Res55_value 0x02  // By pass enable configuration
#define Res106_value 0x00 // user control
#define Res107_value 0x00 // do not reset all the device //0x80

#define Res25 25 //register address
#define Res26 26
#define Res27 27
#define Res28 28
#define Res55 55
#define Res106 106
#define Res107 107

#define start_read_address 59
#define gyro_zero_x -55
#define gyro_zero_y -31
#define gyro_zero_z -123

#define gyro_sensitivity 131.072
#define acce_sensitivity 16384
#define acc_x_gain 16418
#define acc_y_gain 16403
#define acc_z_gain 16600

#define WHILE_TIMEOUT(x) while(x)
/*#define WHILE_TIMEOUT(x)              TIM2->CNT = 0;\*/
                         /*while(x && TIM2->CNT<6000)\*/
//---------------- end of data for MPU 6050 -------------


//-------- variables ----------------------------
extern int16_t gyro_x,gyro_y,gyro_z;
extern int16_t acc_x,acc_y,acc_z;
extern float gyro_x_rate,gyro_y_rate,gyro_z_rate;
extern float acc_x_angle,acc_y_angle;
extern float acc_x_temp,acc_y_temp,acc_z_temp;
extern float z_angle;
extern float dt,temp;
extern uint8_t test,i2c_sdata[20],i2c_rdata[20];
//-----------------------------------------------


//-------------- functions ----------------------
void MPU6050_WriteBlock(uint8_t adr, uint8_t data[], uint8_t data_len);
void MPU6050_ReadBlock(uint8_t adr, uint8_t data[], uint8_t data_len);
void MPU6050_Init(void);
void MPU6050_Read(void);
void MPU_GetGyroRate(void);
void MPU_GetAccValue(void);
//void get_acce_angle(void);
//-----------------------------------------------
#endif
