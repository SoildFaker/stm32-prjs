//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include "conf.h"
#ifndef M_PI
#define M_PI 3.14159265359
#endif

#define roll_angle_off_set  -2.1523
#define pitch_angle_off_set -2.9361
//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern volatile float rpy[3],Eangle[3];
//extern float roll,pitch,yaw;

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void getRollPitchYaw(void);
void getEangle(void);
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
