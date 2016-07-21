#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265359
#endif

#define roll_angle_off_set  -2.1523
#define pitch_angle_off_set -2.9361

// Variable declaration
extern float beta;				// algorithm gain
extern volatile float roll, pitch, yaw;

// Function declarations
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void AHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void AHRS_GetRPY(void);
void AHRS_GetEangle(void);
void AHRS_Init(void);
#endif
