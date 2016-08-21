#include "pid.h"
#include "ahrs.h"
#include "mpu6050.h"
#include "flow.h"
#include "tools.h"
#include "stm32f10x.h"

float rx_value[6] = {
  0.0f,0.0f,0.0f,//roll, pitch, yaw
  0.0f,0.0f,30.0f //x,y,z point
};
float throttle = 0.50f;
float ERRz,_ERRz,ERRzD,PIDz_Out;

float ERRx,_ERRx,ERRxI;
float ERRy,_ERRy,ERRyI;

float ERRxp,_ERRxp,ERRxpI;
float ERRyp,_ERRyp,ERRypI;
float ERRzp,_ERRzp,ERRzpI;

float P_Temp,I_Temp,D_Temp;

float PIDx,PIDy,PIDz;
float PIDxp,PIDyp,PIDzp;
float RateX,RateY,RateZ;

float PID_Value[21]={
  7.077, 0.002, 0.016,//x coordinate PID
  9.640, 0.003, 5.081,//yoll PID
  7.085, 0.002, 0.016,//y coordinate PID
  9.643, 0.003, 5.081,//pitch PID
  0.203, 3.022, 0.016,//z coordinate PID
  8.060, 0.000, 0.007,//yaw PID
  35.00, 35.00, 10.00 //
};

void PIDxp_Update(float dt)
{
  float tempx = 0.03f;
  _ERRxp=ERRxp;
  ERRxp=CONSTRAIN(rx_value[3]-X, MAX_DELTA_X);
  ERRxpI=CONSTRAIN(ERRxp*dt+ERRxpI, I_P_MAX);

  P_Temp=PID_Value[0]*ERRxp;
  I_Temp=PID_Value[1]*ERRxpI;
  D_Temp=CONSTRAIN(-PID_Value[3]*dx,PG_MAX);

  PIDxp=(int)CONSTRAIN((P_Temp+I_Temp+D_Temp),PID_X_MAX);

  rx_value[0]=tempx*PIDxp;
}
void PIDyp_Update(float dt)
{
  float tempy = 0.03f;
  _ERRyp=ERRyp;
  ERRyp=CONSTRAIN(rx_value[4]-Y, MAX_DELTA_Y);
  ERRypI=CONSTRAIN(ERRyp*dt+ERRypI, I_P_MAX);

  P_Temp=PID_Value[6]*ERRyp;
  I_Temp=PID_Value[7]*ERRypI;
  D_Temp=CONSTRAIN(-PID_Value[8]*dy,PG_MAX);

  PIDyp=(int)CONSTRAIN((P_Temp+I_Temp+D_Temp),PID_Y_MAX);
  rx_value[1]=tempy*PIDyp;
}

void PIDzp_Update(float dt)
{
  float tempz = 0.003f;
  _ERRzp=ERRzp;
  ERRzp=CONSTRAIN(rx_value[5]-height, MAX_DELTA_Z);
  ERRzpI=CONSTRAIN(ERRzp*dt+ERRzpI, I_H_MAX);
  RateZ=CONSTRAIN((ERRzp-_ERRzp)/dt,MAX_Z_RATE);

  P_Temp=PID_Value[12]*ERRzp;
  I_Temp=PID_Value[13]*ERRzpI;
  D_Temp=CONSTRAIN(PID_Value[14]*RateZ,PG_MAX);

  PIDzp=(int)MINMAX((P_Temp+I_Temp+D_Temp)+100,100,PID_H_OUT_MAX);
  /*myprintf("ErrZ:%f\tErrZI:%f\tRateZ:%f\t\r\n",ERRzp,ERRzpI,RateZ);*/
  /*myprintf("P:%f\tI:%f\tD:%f\t\r\n",P_Temp,I_Temp,D_Temp);*/
  /*myprintf("PID:%f\r\n",PIDzp);*/
  throttle=tempz*PIDzp;
}

void PIDx_Update(float dt)
{
  _ERRx=ERRx; 
  ERRx=CONSTRAIN(rx_value[0]-roll,MAX_TARGET_ANGLE);
  ERRxI=CONSTRAIN(ERRx*dt+ERRxI,I_MAX);
    
  P_Temp=PID_Value[3]*ERRx;
  I_Temp=PID_Value[4]*ERRxI;
  D_Temp=CONSTRAIN(-PID_Value[5]*gyro_x_rate,PG_MAX); 

  PIDx=(int)CONSTRAIN((P_Temp+I_Temp+D_Temp)/1.414,PID_OUT_MAX);
}

void PIDy_Update(float dt)
{
  _ERRy=ERRy; 
  ERRy=CONSTRAIN(rx_value[1]-pitch,MAX_TARGET_ANGLE);
  ERRyI=CONSTRAIN(ERRy*dt+ERRyI,I_MAX);
    
  P_Temp=PID_Value[9]*ERRy;
  I_Temp=PID_Value[10]*ERRyI;
  D_Temp=CONSTRAIN(-PID_Value[11]*gyro_y_rate,PG_MAX);

  PIDy= (int)CONSTRAIN((P_Temp+I_Temp+D_Temp)/1.414,PID_OUT_MAX);
}

void PIDz_Update(float dt)
{
  if(rx_value[2]==0){
    //auto hold mode, increas PIDz_Out 0.5 by one step to prevent ocilation
    PIDz_Out+=0.5;
    if(PIDz_Out>PID_Z_MAX)PIDz_Out=PID_Z_MAX;
  }else{
    // control z axis, just need small output
    PIDz_Out=PID_Z_MIN;
  }
  _ERRz=ERRz;
  ERRz=CONSTRAIN(rx_value[2]-gyro_z_rate,MAX_GYRO_ERROR);
  ERRzD=(ERRz-_ERRz)/dt;
  
  P_Temp=CONSTRAIN(PID_Value[15]*ERRz,PG_MAX);
  D_Temp=PID_Value[17]*ERRzD;
  
  PIDz= (int) CONSTRAIN(P_Temp+D_Temp,PIDz_Out); 
}

//MT4 (CW)      MT3(CCW)  
//  \     +x     /
//   \    |     /
//    \   |    /  
//     \------/
// +y---******---
//     /------\
//    /   |    \
//   /    |     \
//  /            \
//MT1(CCW)     MT2(CW)
void MORTOR_Output(void)
{
  uint16_t pwm[4];
  uint8_t i;
  if(throttle<0.04)
  {
    TIM4->CCR1=STOP_PWM;//pa0
    TIM4->CCR2=STOP_PWM;//pa1
    TIM4->CCR3=STOP_PWM;//pa2
    TIM4->CCR4=STOP_PWM;//pa3
    GPIOB->ODR&=~(1<<2);
  }
  else
  {   
    pwm[0]=(uint16_t)(throttle*PWM_RANGE)+MIN_PWM+PIDx+PIDy-PIDz-(uint16_t)PID_Value[21];
    pwm[1]=(uint16_t)(throttle*PWM_RANGE)+MIN_PWM-PIDx+PIDy+PIDz-(uint16_t)PID_Value[21];
    pwm[2]=(uint16_t)(throttle*PWM_RANGE)+MIN_PWM-PIDx-PIDy-PIDz+(uint16_t)PID_Value[21]; 
    pwm[3]=(uint16_t)(throttle*PWM_RANGE)+MIN_PWM+PIDx-PIDy+PIDz+(uint16_t)PID_Value[21];

    for(i=0;i<4;i++)
    {
      if(pwm[i]<MIN_PWM)pwm[i]=MIN_PWM;
      if(pwm[i]>MAX_PWM)pwm[i]=MAX_PWM;
    }
    
    TIM4->CCR1=pwm[0];//pa0
    TIM4->CCR2=pwm[1];//pa1
    TIM4->CCR3=pwm[2];//pa2
    TIM4->CCR4=pwm[3];//pa3
    GPIOB->ODR|=1<<2;
  }
}
  

