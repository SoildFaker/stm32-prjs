#include "pid.h"
#include "motor.h"
#include "ahrs.h"
#include "mpu6050.h"
#include "tools.h"
#include "stm32f10x.h"

uint8_t control_mode = 0;
float loop_time = 0.0f;
float height = 0.0f;
float rx_value[6] = {0.00f,0.00f,0.28f,0.0f,0.0f,0.0f};//store the pulse times of 6 input pwm chanels from rx device
									// rx_value[0]: target_x_angle/target_x_rate
									// rx_value[1]: target_y_angle/target_y_rate
									// rx_value[2]: throttle
									// rx_value[3]: target_z_angle/target_z_rate
									// rx_value[4]: (unused)
									// rx_value[5]: (unused)
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


/*
	PID gain value setup
*/
void PID_SetGainValue(void)
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

/*
	Get PID gain value from Processing
*/
void PID_GetGainValue(void)
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

/*
	Send pid_gain data to Processing
*/
void PID_SendGain(void)
{
	if(read_request)
	{
	uint8_t i;  
  for(i=0;i<3;i++)
  {
    temp_gain_vl=(int)(pid_gain_vl[page*3+i]*1000);
    temp_data[3*i]=temp_gain_vl/10000;
    temp_data[3*i+1]=(temp_gain_vl/100)%100;
    temp_data[3*i+2]=temp_gain_vl % 100;
  }
	
	//------ send data to Processing via USART3 -------
	/*USART3_put_char(255);//start byte*/
	for(i=0;i<9;i++)
	{
		/*USART3_put_char(temp_data[i]);*/
	}
	//-------------------------------------------------
	read_request=0;
	}
}
/*
	PID update for X axis
*/
void PIDx_Update(void)
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
  PID_x=(int)CONSTRAIN((Pg_temp+Ig_temp+Dg_temp)/1.414,PID_OUT_MAX);
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
  PID_x=(int)CONSTRAIN((P_imu_temp+I_imu_temp+D_imu_temp)/1.414,PID_OUT_MAX);
	//----------------------------------------------
	}
}
/*
	PID update for Y axis
*/
void PIDy_Update(void)
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
  PID_y=(int)CONSTRAIN((Pg_temp+Ig_temp+Dg_temp)/1.414,PID_OUT_MAX);
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
  PID_y= (int)CONSTRAIN((P_imu_temp+I_imu_temp+D_imu_temp)/1.414,PID_OUT_MAX);
	//----------------------------------------------
	}
}

/*
	PID update for z axis
*/
void PIDz_Update(void)
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

void MORTOR_Output(void)
{
//////Quad X configuration
//////MT4 (CW)			MT3(CCW)	
//////	\			+x		 /
//////	 \		|			/
//////		\		|		 /	
//////		 \------/
////// +y---******---
//////		 /------\
//////		/		|		 \
//////	 /		|			\
//////	/						 \
//////MT1(CCW)	   MT2(CW)
	
	uint16_t temp_pwm[4];
	uint8_t i;
	if(rx_value[2]<0.04)
	{
		TIM4->CCR1=STOP_PWM;//pa0
		TIM4->CCR2=STOP_PWM;//pa1
		TIM4->CCR3=STOP_PWM;//pa2
		TIM4->CCR4=STOP_PWM;//pa3
		GPIOB->ODR&=~(1<<2);
		//printf("stop\r");
	}
	else
	{		
		temp_pwm[0]=(uint16_t)(rx_value[2]*PWM_RANGE)+MIN_PWM+PID_x+PID_y+PID_z-(uint16_t)pid_gain_vl[17];
		temp_pwm[1]=(uint16_t)(rx_value[2]*PWM_RANGE)+MIN_PWM-PID_x+PID_y-PID_z-(uint16_t)pid_gain_vl[17];
		temp_pwm[2]=(uint16_t)(rx_value[2]*PWM_RANGE)+MIN_PWM-PID_x-PID_y+PID_z+(uint16_t)pid_gain_vl[17]; 
		temp_pwm[3]=(uint16_t)(rx_value[2]*PWM_RANGE)+MIN_PWM+PID_x-PID_y-PID_z+(uint16_t)pid_gain_vl[17];
		//printf("quad X\r");

		for(i=0;i<4;i++)
		{
			if(temp_pwm[i]<MIN_PWM)temp_pwm[i]=MIN_PWM;
			if(temp_pwm[i]>MAX_PWM)temp_pwm[i]=MAX_PWM;
		}
		
    TIM4->CCR1=temp_pwm[0];//pa0
    TIM4->CCR2=temp_pwm[1];//pa1
    TIM4->CCR3=temp_pwm[2];//pa2
    TIM4->CCR4=temp_pwm[3];//pa3
    float height_temp = HCSR04_Get();
    if(height_temp < 30){
      if(height_temp > height){
        rx_value[2] = rx_value[2]*0.99f;
      }else{
        rx_value[2] = rx_value[2]*1.01f;
      }
      if(rx_value[2]>0.8f){
        rx_value[2] = 0.8f;
      }
    }else{
      if(height_temp > height){
        rx_value[2] = rx_value[2]*0.99f;
      }else{
        rx_value[2] = rx_value[2]*1.01f;
      }
      if(rx_value[2]<0.3f){
        rx_value[2] = 0.3f;
      }
    }
    height = HCSR04_Get();
		//printf("running\r");
		GPIOB->ODR|=1<<2;
	}
	/*
	if(rx_value[2]>0.0)
	{
		temp_pwm[0]=(uint16_t)(rx_value[2]*5000);
		TIM4->CCR1=temp_pwm[0];//pa0
		TIM4->CCR2=temp_pwm[0];//pa1
		TIM4->CCR3=temp_pwm[0];//pa2
		TIM4->CCR4=temp_pwm[0];//pa3
		printf("%0.1f %d \r",rx_value[2],temp_pwm[0]);
	}
	*/
}
	




