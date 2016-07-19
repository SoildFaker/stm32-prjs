#include "mpu6050.h"
#include "tools.h"
#include "i2c.h"

int accX, accY, accZ;
int gyroX, gyroY, gyroZ;
//**************************************
//初始化MPU6050
//**************************************
void MPU6050_Init()
{
  Single_WriteI2C(MPU6050_Addr, PWR_MGMT_1, 0x00);  //解除休眠状态
  Single_WriteI2C(MPU6050_Addr, SMPLRT_DIV, 0x07);// Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  Single_WriteI2C(MPU6050_Addr, CONFIG, 0x00);// Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  Single_WriteI2C(MPU6050_Addr, GYRO_CONFIG, 0x00);// Set Gyro Full Scale Range to ±250deg/s
  Single_WriteI2C(MPU6050_Addr, ACCEL_CONFIG, 0x00);// Set Accelerometer Full Scale Range to ±2g
  Single_WriteI2C(MPU6050_Addr, PWR_MGMT_1, 0x01);// PLL with X axis gyroscope reference and disable sleep mode
}
//**************************************
//// Get accelerometer and gyroscope values
//**************************************
void MPU6050_Update()
{
  accX=((Single_ReadI2C(MPU6050_Addr, ACCEL_XOUT_H)<<8)+Single_ReadI2C(MPU6050_Addr, ACCEL_XOUT_L));
  accY=-((Single_ReadI2C(MPU6050_Addr, ACCEL_YOUT_H)<<8)+Single_ReadI2C(MPU6050_Addr, ACCEL_YOUT_L));
  accZ=((Single_ReadI2C(MPU6050_Addr, ACCEL_ZOUT_H)<<8)+Single_ReadI2C(MPU6050_Addr, ACCEL_ZOUT_L));
  
      myprintf("roll:%d\tpitch:%d\tyaw:%d\r\n", accX, accZ, accY);
  gyroX=-((Single_ReadI2C(MPU6050_Addr, GYRO_XOUT_H)<<8)+Single_ReadI2C(MPU6050_Addr, GYRO_XOUT_L));
  gyroY=((Single_ReadI2C(MPU6050_Addr, GYRO_YOUT_H)<<8)+Single_ReadI2C(MPU6050_Addr, GYRO_YOUT_L));
  gyroZ=-((Single_ReadI2C(MPU6050_Addr, GYRO_ZOUT_H)<<8)+Single_ReadI2C(MPU6050_Addr, GYRO_ZOUT_L));  
}
