#ifndef _KALMAN_X_H_
#define _KALMAN_X_H_
extern float Q_angleX ; 
extern float Q_gyroX  ;  
extern float R_angleX ;  

extern float x_angle;
extern float x_bias;
extern float PX_00, PX_01, PX_10, PX_11;	
extern float dtX, yX, SX;
extern float KX_0, KX_1;
		
float kalman_filter(float newAngle, float newRate,float dtX);
#endif
