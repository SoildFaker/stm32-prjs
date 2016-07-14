#include "i2c.h"
#include "mpu6050.h"
//**************************************
//初始化MPU6050
//**************************************
void MPU6050_Init()
{
    SlaveAddress=MPU6050_Addr;
    Single_WriteI2C(PWR_MGMT_1, 0x00);    //解除休眠状态
    Single_WriteI2C(SMPLRT_DIV, 0x07);// Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    Single_WriteI2C(CONFIG, 0x00);// Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    Single_WriteI2C(GYRO_CONFIG, 0x00);// Set Gyro Full Scale Range to ±250deg/s
    Single_WriteI2C(ACCEL_CONFIG, 0x00);// Set Accelerometer Full Scale Range to ±2g
    Single_WriteI2C(PWR_MGMT_1, 0x01);// PLL with X axis gyroscope reference and disable sleep mode
}
//**************************************
//// Get accelerometer and gyroscope values
//**************************************
void MPU6050_Update()
{
    SlaveAddress=MPU6050_Addr;// Get accelerometer and gyroscope values

    accX=((Single_ReadI2C(ACCEL_XOUT_H)<<8)+Single_ReadI2C(ACCEL_XOUT_L));
    accY=-((Single_ReadI2C(ACCEL_YOUT_H)<<8)+Single_ReadI2C(ACCEL_YOUT_L));
    accZ=((Single_ReadI2C(ACCEL_ZOUT_H)<<8)+Single_ReadI2C(ACCEL_ZOUT_L));
    
    gyroX=-((Single_ReadI2C(GYRO_XOUT_H)<<8)+Single_ReadI2C(GYRO_XOUT_L));
    gyroY=((Single_ReadI2C(GYRO_YOUT_H)<<8)+Single_ReadI2C(GYRO_YOUT_L));
    gyroZ=-((Single_ReadI2C(GYRO_ZOUT_H)<<8)+Single_ReadI2C(GYRO_ZOUT_L));    
}
