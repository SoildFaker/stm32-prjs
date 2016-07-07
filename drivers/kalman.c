// KasBot V2  -  Kalman filter module - http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1284738418 - http://www.x-firm.com/?page_id=145
//with slightly modifications by Kristian Lauszus
#include "kalman.h"
float Q_angleX  =  0.001; 
float Q_gyroX   =  0.003;  
float R_angleX  =  0.03;  

float x_angle = 0;
float x_bias = 0;
float PX_00 = 0, PX_01 = 0, PX_10 = 0, PX_11 = 0;	
float dtX, yX, SX;
float KX_0, KX_1;
	
float kalman_filter(float newAngle, float newRate,float dtX)
{                          
  x_angle += dtX * (newRate - x_bias);
  PX_00 +=  - dtX * (PX_10 + PX_01) + Q_angleX * dtX;
  PX_01 +=  - dtX * PX_11;
  PX_10 +=  - dtX * PX_11;
  PX_11 +=  + Q_gyroX * dtX;
  
  yX = newAngle - x_angle;
  SX = PX_00 + R_angleX;
  KX_0 = PX_00 / SX;
  KX_1 = PX_10 / SX;
  
  x_angle +=  KX_0 * yX;
  x_bias  +=  KX_1 * yX;
  PX_00 -= KX_0 * PX_00;
  PX_01 -= KX_0 * PX_01;
  PX_10 -= KX_1 * PX_00;
  PX_11 -= KX_1 * PX_01;

  return x_angle;
}
	
	
	
