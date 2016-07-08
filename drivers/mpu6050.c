#include "conf.h"
#include "tools.h" 
#define PRINT_ACCEL   (0x01)
#define PRINT_GYRO  (0x02)
#define PRINT_QUAT  (0x04)
#define ACCEL_ON  (0x01)
#define GYRO_ON   (0x02)
#define MOTION    (0)
#define NO_MOTION   (1)
#define DEFAULT_MPU_HZ  (200)
#define FLASH_SIZE  (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f
short gyro[3], accel[3], sensors;
float Pitch,Roll,Yaw ; 
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
static signed char gyro_orientation[9] = {-1, 0, 0,
             0,-1, 0,
             0, 0, 1};

static  unsigned short inv_row_2_scale(const signed char *row)
{
  unsigned short b;

  if (row[0] > 0)
  b = 0;
  else if (row[0] < 0)
  b = 4;
  else if (row[1] > 0)
  b = 1;
  else if (row[1] < 0)
  b = 5;
  else if (row[2] > 0)
  b = 2;
  else if (row[2] < 0)
  b = 6;
  else
  b = 7;  // error
  return b;
}


static  unsigned short inv_orientation_matrix_to_scalar(
  const signed char *mtx)
{
  unsigned short scalar;
  scalar = inv_row_2_scale(mtx);
  scalar |= inv_row_2_scale(mtx + 3) << 3;
  scalar |= inv_row_2_scale(mtx + 6) << 6;


  return scalar;
}

static void run_self_test(void)
{
  int result;
  long gyro[3], accel[3];

  result = mpu_run_self_test(gyro, accel);
  if (result == 0x7) {
  /* Test passed. We can trust the gyro data here, so let's push it down
   * to the DMP.
   */
  float sens;
  unsigned short accel_sens;
  mpu_get_gyro_sens(&sens);
  gyro[0] = (long)(gyro[0] * sens);
  gyro[1] = (long)(gyro[1] * sens);
  gyro[2] = (long)(gyro[2] * sens);
  dmp_set_gyro_bias(gyro);
  mpu_get_accel_sens(&accel_sens);
  accel[0] *= accel_sens;
  accel[1] *= accel_sens;
  accel[2] *= accel_sens;
  dmp_set_accel_bias(accel);
  myprintf("setting bias succesfully ......\r\n");
  }
}



uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;


/**************************实现函数********************************************
*函数原型:  void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
*功　　能:  将新的ADC数据更新到 FIFO数组，进行滤波处理
*******************************************************************************/
void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
unsigned char i ;
int32_t sum=0;
for(i=1;i<10;i++){  //FIFO 操作
MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
}
MPU6050_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
MPU6050_FIFO[1][9]=ay;
MPU6050_FIFO[2][9]=az;
MPU6050_FIFO[3][9]=gx;
MPU6050_FIFO[4][9]=gy;
MPU6050_FIFO[5][9]=gz;

sum=0;
for(i=0;i<10;i++){  //求当前数组的合，再取平均值
   sum+=MPU6050_FIFO[0][i];
}
MPU6050_FIFO[0][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[1][i];
}
MPU6050_FIFO[1][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[2][i];
}
MPU6050_FIFO[2][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[3][i];
}
MPU6050_FIFO[3][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[4][i];
}
MPU6050_FIFO[4][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[5][i];
}
MPU6050_FIFO[5][10]=sum/10;
}

uint8_t MPU6050_getDeviceID(void) {

  i2cRead(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
  return buffer[0];
}

uint8_t MPU6050_testConnection(void) {
  if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
  return 1;
  else return 0;
}

void MPU6050_Init(void) {
  uint8_t i2c_sdata[4];
	//reset mpu6050
	i2c_sdata[0]=0x80;
	i2cWrite(devAddr, Res107, 1, i2c_sdata);
	DelayMs(10);
	
	//use internal clock
	i2c_sdata[0]=Res107_value;
	i2cWrite(devAddr, Res107, 1, i2c_sdata);
	DelayMs(10);
	
	// enable i2c master mode, arm uc will contrl the mpu6050 bypass auxilary bus
	i2c_sdata[0]=Res106_value;
	i2cWrite(devAddr, Res106, 1, i2c_sdata);
	DelayMs(10);
	
	//enable bypass i2c bus
	i2c_sdata[0]=Res55_value;
	i2cWrite(devAddr, Res55, 1, i2c_sdata);
	DelayMs(10);
	
	//config our meure mode, read the specific res for more info
	i2c_sdata[0]=Res25_value;
	i2c_sdata[1]=Res26_value;
	i2c_sdata[2]=Res27_value;
	i2c_sdata[3]=Res28_value;
	i2cWrite(devAddr, Res25, 4, i2c_sdata);
	DelayMs(10);
	//printf("mpu6050 init done\r");
}




/**************************************************************************
函数功能：MPU6050内置DMP的初始化
入口参数：无
返回  值：无
**************************************************************************/
void DMP_Init(void)
{ 
  u8 temp[1]={0};
  i2cRead(0x68,0x75,1,temp);
  myprintf("mpu_set_sensor complete ......\r\n");
  /*if(temp[0]!=0x68)NVIC_SystemReset();*/
  if(!mpu_init(0))
  {
  if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
     myprintf("mpu_set_sensor complete ......\r\n");
  if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
     myprintf("mpu_configure_fifo complete ......\r\n");
  if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
     myprintf("mpu_set_sample_rate complete ......\r\n");
  if(!dmp_load_motion_driver_firmware())
    myprintf("dmp_load_motion_driver_firmware complete ......\r\n");
  if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
     myprintf("dmp_set_orientation complete ......\r\n");
  if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
    DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
    DMP_FEATURE_GYRO_CAL))
     myprintf("dmp_enable_feature complete ......\r\n");
  if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
     myprintf("dmp_set_fifo_rate complete ......\r\n");
  run_self_test();
  if(!mpu_set_dmp_state(1))
     myprintf("mpu_set_dmp_state complete ......\r\n");
  }
}

void Read_DMP(void)
{ 
  unsigned long sensor_timestamp;
  unsigned char more;
  long quat[4];

    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);   
    if ( sensors & INV_WXYZ_QUAT )
    {  
      q0=(float)quat[0] / q30;
      q1=(float)quat[1] / q30;
      q2=(float)quat[2] / q30;
      q3=(float)quat[3] / q30;
      Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
      Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll   
      Yaw =  atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
    }

}

int Read_Temperature(void)
{  
  float Temp;
  Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
  if(Temp>32768) Temp-=65536;
  Temp=(36.53+Temp/340)*10;
  return (int)Temp;
}
//------------------End of File----------------------------
