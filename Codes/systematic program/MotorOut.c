#include "common.h"
#include "include.h"
#include "self_include.h"


void MotorOutput()
{
	if( ( exceptionflag1==1||stopflag==1 ) )   //&& ( Val_Left<-30 && Val_Right<-30 )   )
	{
		Speed_L=0;
		Speed_R=0;
	}
	else
	{
		Speed_L= /**/-g_fDirectionControlOut-g_fSpeedControl_L-g_fAngleControlOut;// Total output speed of the left wheel      g_fSpeedControl_L
		Speed_R= /**/+g_fDirectionControlOut-g_fSpeedControl_R-g_fAngleControlOut;// Total output speed of the right wheel      g_fSpeedControl_R
	}
	SpeedPWMOutput();
}

/***********************************************************************/
/*
*  Speed control PWM output to motors (including limiting, PWM inversion)
*/
/******************************************************************************/
void SpeedPWMOutput()
{
	/*********************** Limit maximum speed to 500 PWM ******************************/
	if(Speed_L > 500)  Speed_L = 500;
	if(Speed_L < -500) Speed_L = -500;     //500
	if(Speed_R >500)  Speed_R = 500;
	if(Speed_R < -500) Speed_R = -500;
	//    /*************** Because the driver is equipped with an inverter, a final processing of the speed is required. ******************/
	//    if(Speed_L > 0)     // Because the inverter is added, the PWM is added in reverse.
	//        Speed_L_Last = 500 - Speed_L;
	//    else
	//        Speed_L_Last = -500 - Speed_L;
	//
	//    if(Speed_R > 0)     // Because the inverter is added, the PWM is added in reverse.
	//        Speed_R_Last = 500 - Speed_R;
	//    else
	//        Speed_R_Last = -500 - Speed_R;

	/************* PWM control with the speed of the corresponding angle obtained ********************/
	if(Speed_L >= 0)    // Angle is greater than 0, forward; less than 0, backward
	{
		ftm_pwm_duty(FTM0,FTM_CH5,(uint32)(Speed_L + MOTOR_DEAD_BACK_L));
		ftm_pwm_duty(FTM0,FTM_CH6,0);    // Add dead zone voltage
	}
	else
	{
		ftm_pwm_duty(FTM0,FTM_CH6,(uint32)(-Speed_L + MOTOR_DEAD_FORE_L));
		ftm_pwm_duty(FTM0,FTM_CH5,0);    // Add dead zone voltage
	}

	if(Speed_R >= 0)    // Angle is greater than 0, forward; less than 0, backward
	{
		ftm_pwm_duty(FTM0,FTM_CH3,(uint32)(Speed_R + MOTOR_DEAD_BACK_R));
		ftm_pwm_duty(FTM0,FTM_CH4,0);    // Add dead zone voltage
	}
	else
	{
		ftm_pwm_duty(FTM0,FTM_CH4,(uint32)(-Speed_R + MOTOR_DEAD_FORE_R));
		ftm_pwm_duty(FTM0,FTM_CH3,0);   // Add dead zone voltage
	}
}
