#include "pid.h"
#include "ahrs.h"
#include <stdio.h>
#include "conf.h"
//------- variables -------------------------------
float pid_gain_vl[18];
/*
  pid_gain_vl[0]=Kg_x_P
  pid_gain_vl[1]=Kg_x_I
  pid_gain_vl[2]=Kg_x_D
  
  pid_gain_vl[3]=KI_x_P
  pid_gain_vl[4]=KI_x_I
  pid_gain_vl[5]=KI_x_D
  
  pid_gain_vl[6]=Kg_y_P
  pid_gain_vl[7]=Kg_y_I
  pid_gain_vl[8]=Kg_y_D
  
  pid_gain_vl[9]=KI_y_P
  pid_gain_vl[10]=KI_y_I
  pid_gain_vl[11]=KI_y_D
  
  pid_gain_vl[12]=Kg_z_P
  pid_gain_vl[13]=Kg_z_I
  pid_gain_vl[14]=Kg_z_D
  
  pid_gain_vl[15]=Trim_value_x_axis
  pid_gain_vl[16]=Trim_value_y_axis
  pid_gain_vl[17]=Trim_value_x_axis_X_mode
*/
uint8_t page,pid_pt,pid_temp[9],read_request;
int temp_data[9];
int temp_gain_vl;

float gyro_x_errorI,gyro_y_errorI;
float gyro_x_error,last_gyro_x_error,delta_gyro_x_error,x_rate_temp;
float gyro_y_error,last_gyro_y_error,delta_gyro_y_error,y_rate_temp;;
float gyro_z_error,last_gyro_z_error,delta_gyro_z_error,pid_z_out_temp;

float imu_x_error,imu_x_target,last_imu_x_error,imu_x_errorI;
float imu_y_error,imu_y_target,last_imu_y_error,imu_y_errorI;

float Pg_temp,Ig_temp,Dg_temp;
float P_imu_temp,I_imu_temp,D_imu_temp;

float PID_x,PID_y,PID_z;
//-------------------------------------------------


//------------- functions -------------------------
/* PID gain value setup */
void Set_pid_gain_value(void)
{
  pid_gain_vl[0]=7.077;
  pid_gain_vl[1]=0.002;
  pid_gain_vl[2]=0.016;
  
  pid_gain_vl[3]=5.640;
  pid_gain_vl[4]=0.003;
  pid_gain_vl[5]=4.081;
  
  pid_gain_vl[6]=7.085;
  pid_gain_vl[7]=0.002;
  pid_gain_vl[8]=0.016;
  
  pid_gain_vl[9]=5.643;
  pid_gain_vl[10]=0.003;
  pid_gain_vl[11]=4.081;
  
  pid_gain_vl[12]=8.06;
  pid_gain_vl[13]=0;
  pid_gain_vl[14]=0.007;
  
  pid_gain_vl[15]=35;//30.0;
  pid_gain_vl[16]=35;//25.0;
  pid_gain_vl[17]=10;
}

/* Get PID gain value from Processing */
void Get_pid_gain_value(void)
{
  if(pid_pt>8)
  {
  uint8_t i;
  for(i=0;i<3;i++)
  {
  pid_gain_vl[3*page+i]=(float)((pid_temp[3*i]*1e4+pid_temp[3*i+1]*1e2+pid_temp[3*i+2])/1000);
  }
  pid_pt=0;
  }
}

/*[> Send pid_gain data to Processing <]*/
/*void Send_pid_gain(void)*/
/*{*/
  /*if(read_request)*/
  /*{*/
  /*uint8_t i;  */
  /*for(i=0;i<3;i++)*/
  /*{*/
    /*temp_gain_vl=(int)(pid_gain_vl[page*3+i]*1000);*/
    /*temp_data[3*i]=temp_gain_vl/10000;*/
    /*temp_data[3*i+1]=(temp_gain_vl/100)%100;*/
    /*temp_data[3*i+2]=temp_gain_vl % 100;*/
  /*}*/
  
  /*//------ send data to Processing via USART3 -------*/
  /*USART3_put_char(255);//start byte*/
  /*for(i=0;i<9;i++)*/
  /*{*/
    /*USART3_put_char(temp_data[i]);*/
  /*}*/
  /*//-------------------------------------------------*/
  /*read_request=0;*/
  /*}*/
/*}*/
/* PID update for X axis */
void PID_x_update(void)
{
  if(control_mode)
  {
  //------ update PID value for gyroscope -------
  last_gyro_x_error=gyro_x_error;
  gyro_x_error=CONSTRAIN(rx_value[0]-gyro_x_rate,MAX_GYRO_ERROR);     
  gyro_x_errorI=CONSTRAIN(gyro_x_error*loop_time+gyro_x_errorI,IG_MAX); 
  delta_gyro_x_error=(gyro_x_error-last_gyro_x_error)/loop_time;
    
  Pg_temp=CONSTRAIN(pid_gain_vl[0]*gyro_x_error,PG_MAX);
  Ig_temp=pid_gain_vl[1]*gyro_x_errorI;
  Dg_temp=pid_gain_vl[2]*delta_gyro_x_error;
  //--------------------------------------------- 
  
  //------- calculate PID out value for X axis ---
  #ifdef _QUAD_PLUS_
    PID_x=(int)CONSTRAIN(Pg_temp+Ig_temp+Dg_temp,PID_OUT_MAX);
  #endif
  
  #ifdef _QUAD_X_
    PID_x=(int)CONSTRAIN((Pg_temp+Ig_temp+Dg_temp)/1.414,PID_OUT_MAX);
  #endif
  //----------------------------------------------
  }
  else// angle mode
  {
  //------ update PID value for imu       -------
  last_imu_x_error=imu_x_error; 
  imu_x_error=CONSTRAIN(rx_value[0]-rpy[0],MAX_TARGET_ANGLE);
  imu_x_errorI=CONSTRAIN(imu_x_error*loop_time+imu_x_errorI,I_MAX);
  //x_rate_temp=((imu_x_error-last_imu_x_error)/loop_time-gyro_x_rate*3)/4;//combine gyro data and angle_error_data for D controller
    
  P_imu_temp=pid_gain_vl[3]*imu_x_error;
  I_imu_temp=pid_gain_vl[4]*imu_x_errorI;
  //D_imu_temp=CONSTRAIN(pid_gain_vl[5]*x_rate_temp,PG_MAX);  
  D_imu_temp=CONSTRAIN(-pid_gain_vl[5]*gyro_x_rate,PG_MAX); 
  //------- calculate PID out value for X axis ---
  #ifdef _QUAD_PLUS_
    PID_x=(int)CONSTRAIN(P_imu_temp+I_imu_temp+D_imu_temp,PID_OUT_MAX);
  #endif
    
  #ifdef _QUAD_X_
    PID_x=(int)CONSTRAIN((P_imu_temp+I_imu_temp+D_imu_temp)/1.414,PID_OUT_MAX);
  #endif
  //----------------------------------------------
  }
}
/* PID update for Y axis */
void PID_y_update(void)
{
  if(control_mode)
  {
  //------ update PID value for gyroscope -------
  last_gyro_y_error=gyro_y_error;
  gyro_y_error=CONSTRAIN(rx_value[1]-gyro_y_rate,MAX_GYRO_ERROR);     
  gyro_y_errorI=CONSTRAIN(gyro_y_error*loop_time+gyro_y_errorI,IG_MAX); 
  delta_gyro_y_error=(gyro_y_error-last_gyro_y_error)/loop_time;
    
  Pg_temp=CONSTRAIN(pid_gain_vl[6]*gyro_y_error,PG_MAX);
  Ig_temp=pid_gain_vl[7]*gyro_y_errorI;
  Dg_temp=pid_gain_vl[8]*delta_gyro_y_error;
  //--------------------------------------------- 
  
  //------- calculate PID out value for Y axis ---
  #ifdef _QUAD_PLUS_
    PID_y=(int)CONSTRAIN(Pg_temp+Ig_temp+Dg_temp,PID_OUT_MAX);
  #endif
    
  #ifdef _QUAD_X_
    PID_y=(int)CONSTRAIN((Pg_temp+Ig_temp+Dg_temp)/1.414,PID_OUT_MAX);
  #endif
  //----------------------------------------------
  }
  else// angle mode
  {
  //------ update PID value for imu       -------
  last_imu_y_error=imu_y_error; 
  imu_y_error=CONSTRAIN(rx_value[1]-rpy[1],MAX_TARGET_ANGLE);
  imu_y_errorI=CONSTRAIN(imu_y_error*loop_time+imu_y_errorI,I_MAX);
  //y_rate_temp=((imu_y_error-last_imu_y_error)/loop_time-gyro_y_rate*3)/4;//combine gyro data and angle_error_data for D controller
    
  P_imu_temp=pid_gain_vl[9]*imu_y_error;
  I_imu_temp=pid_gain_vl[10]*imu_y_errorI;
  //D_imu_temp=CONSTRAIN(pid_gain_vl[11]*y_rate_temp,PG_MAX);
  D_imu_temp=CONSTRAIN(-pid_gain_vl[11]*gyro_y_rate,PG_MAX);
  //---------------------------------------------
  
  //------- calculate PID out value for Y axis ---
  #ifdef _QUAD_PLUS_
    PID_y= (int)CONSTRAIN(P_imu_temp+I_imu_temp+D_imu_temp,PID_OUT_MAX);
  #endif
  
  #ifdef _QUAD_X_
    PID_y= (int)CONSTRAIN((P_imu_temp+I_imu_temp+D_imu_temp)/1.414,PID_OUT_MAX);
  #endif
  //----------------------------------------------
  }
}

/* PID update for z axis */
void PID_z_update(void)
{
  if(rx_value[3]==0)//auto hold mode, increas pid_z_out_temp 0.5 by one step to prevent ocilation
  {
    pid_z_out_temp+=1;
    if(pid_z_out_temp>PID_Z_MAX)pid_z_out_temp=PID_Z_MAX;
  }
  else// control z axis, just need small output
  {
    pid_z_out_temp=PID_Z_MIN;
  }
  //--------- update PD value for z gyro -----------------------
  last_gyro_z_error=gyro_z_error;
  gyro_z_error=CONSTRAIN(rx_value[3]-gyro_z_rate,MAX_GYRO_ERROR);
  delta_gyro_z_error=(gyro_z_error-last_gyro_z_error)/loop_time;
  
  Pg_temp=CONSTRAIN(pid_gain_vl[12]*gyro_z_error,PG_MAX);
  Dg_temp=pid_gain_vl[14]*delta_gyro_z_error;
  //------------------------------------------------------------
  
  PID_z= (int) CONSTRAIN(Pg_temp+Dg_temp,pid_z_out_temp); 
}

void MORTOR_output(void)
{
//////Quad X configuration
//////MT4 (CW)      MT3(CCW)  
//////  \     +x     /
//////   \    |     /
//////    \   |    /  
//////     \------/
////// +y---******---
//////     /------\
//////    /   |    \
//////   /    |     \
//////  /            \
//////MT1(CCW)     MT2(CW)
  
  //Quad + configuration  
//           mt1(CW)
//         ***(x)***
//             ^
//             |
//             |
//             |
//             |
//mt2(y)<------0---------mt4
//(CCW)        |    (CCW) 
//             |
//             |
//             |
//           mt3(CW)
  uint16_t temp_pwm[4];
  uint8_t i;
  if(rx_value[2]<0.04)
  {
    TIM2->CCR1=STOP_PWM;//pa0
    TIM2->CCR2=STOP_PWM;//pa1
    TIM2->CCR3=STOP_PWM;//pa2
    TIM2->CCR4=STOP_PWM;//pa3
    GPIOB->ODR&=~(1<<2);
    //printf("stop\r");
  }
  else
  {   
    #ifdef _QUAD_PLUS_
    temp_pwm[0]=(uint16_t)(rx_value[2]*PWM_RANGE)+MIN_PWM-PID_y+PID_z-(uint16_t)pid_gain_vl[16];//trim this motor
    temp_pwm[1]=(uint16_t)(rx_value[2]*PWM_RANGE)+MIN_PWM+PID_x-PID_z-(uint16_t)pid_gain_vl[15];//trim this motor 
    temp_pwm[2]=(uint16_t)(rx_value[2]*PWM_RANGE)+MIN_PWM+PID_y+PID_z+(uint16_t)pid_gain_vl[16];//trim this motor 
    temp_pwm[3]=(uint16_t)(rx_value[2]*PWM_RANGE)+MIN_PWM-PID_x-PID_z+(uint16_t)pid_gain_vl[15];//trim this motor 
    //printf("quad +\r");
    #endif
    
    #ifdef _QUAD_X_
    temp_pwm[0]=(uint16_t)(rx_value[2]*PWM_RANGE)+MIN_PWM+PID_x+PID_y+PID_z-(uint16_t)pid_gain_vl[17];
    temp_pwm[1]=(uint16_t)(rx_value[2]*PWM_RANGE)+MIN_PWM-PID_x+PID_y-PID_z-(uint16_t)pid_gain_vl[17];
    temp_pwm[2]=(uint16_t)(rx_value[2]*PWM_RANGE)+MIN_PWM-PID_x-PID_y+PID_z+(uint16_t)pid_gain_vl[17]; 
    temp_pwm[3]=(uint16_t)(rx_value[2]*PWM_RANGE)+MIN_PWM+PID_x-PID_y-PID_z+(uint16_t)pid_gain_vl[17];
    //printf("quad X\r");
    #endif

    for(i=0;i<4;i++)
    {
      if(temp_pwm[i]<MIN_PWM)temp_pwm[i]=MIN_PWM;
      if(temp_pwm[i]>MAX_PWM)temp_pwm[i]=MAX_PWM;
    }
    
    TIM2->CCR1=temp_pwm[0];//pa0
    TIM2->CCR2=temp_pwm[1];//pa1
    TIM2->CCR3=temp_pwm[2];//pa2
    TIM2->CCR4=temp_pwm[3];//pa3
    //printf("running\r");
    GPIOB->ODR|=1<<2;
  }
  /*
  if(rx_value[2]>0.0)
  {
    temp_pwm[0]=(uint16_t)(rx_value[2]*5000);
    TIM2->CCR1=temp_pwm[0];//pa0
    TIM2->CCR2=temp_pwm[0];//pa1
    TIM2->CCR3=temp_pwm[0];//pa2
    TIM2->CCR4=temp_pwm[0];//pa3
    printf("%0.1f %d \r",rx_value[2],temp_pwm[0]);
  }
  */
}
//-----------------------right motor done --------------------------------
  

//-----usart3 intr service prg-------------
void USART3_IRQHandler(void)//when we have an interrupt on USART
{
   if((USART_GetITStatus(USART3,USART_IT_RXNE))!=RESET)//if match receive complete interrupt
   {
     USART_ClearITPendingBit(USART3,USART_IT_RXNE);//clear the flag bit for the next time
     //-------------------------------------     
     rx3_temp= USART_ReceiveData(USART3); 
     
     if(rx3_temp>209)// reading pid gain request
     {
       page=rx3_temp-210;
       read_request=1;
     }
     else if(rx3_temp>199 && rx3_temp<210)// sending pid gain request
     {
       page=rx3_temp-200;
       pid_pt=0;
     }
     else// pid gain temp data from Processing
     {
       pid_temp[pid_pt]=rx3_temp;
       pid_pt++;
        
     }
   }
   
}
 
//----------------+-----------------------




