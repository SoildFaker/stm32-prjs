#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265359
#endif

#define roll_angle_off_set  -2.1523
#define pitch_angle_off_set -2.9361

// Variable declaration
extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern volatile float rpy[3],Eangle[3];

// Function declarations
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void AHRS_GetRPY(void);
void AHRS_GetEangle(void);
#endif
