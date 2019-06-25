#include "common.h"
#include "include.h"
#include "self_include.h"



//**************************************************************************
/*
*  Function Description:AD Acquisition
*  Parameter description: None
*  Function return: unsigned result value
*/
//**************************************************************************
void Rd_Ad_Value(void)
{
	MMA7361 = adc_once(ZOUT, ADC_12bit);   // Z
	ENC03= adc_once(Gyro1,ADC_12bit);      // gyro1
}

//**************************************************************************
//   Tsinghua angle filtering scheme
//*************************************************************************
/*
*  Function Description: Tsinghua angle filtering
*  Parameter description:G_angle                       Accelerometer angle£¬ 0-90
*            			 Gyro                          Gyro angular velocity converted value
*            			 GRAVITY_ADJUST_TIME_CONSTANT  Time correction factor
*            			 DT                            Timer time /s
*  Function return: unsigned result value
*/
//*************************************************************************
void QingHua_AngleCalaulate(float G_angle,float Gyro)
{
	float fDeltaValue;
	g_fCarAngle = g_fGyroscopeAngleIntegral;   // Final angle
	fDeltaValue = (G_angle - g_fCarAngle) / GRAVITY_ADJUST_TIME_CONSTANT;  // Time coefficient correction
	g_fGyroscopeAngleIntegral += (-Gyro + fDeltaValue) * DT;               // Final angle
    
}

//**************************************************************************
/*
*  Function Description: self-balancing angle calculate
*  Parameter description: None
*  Function return: unsigned result value
*/
//**************************************************************************
void AD_Calculate(void)
{
	Rd_Ad_Value();                          // AD Acquisition

	Gyro_Now = ( GYRO_VAL - ENC03 ) * Gyro_ratio;                            // Normalized angular velocity acquired by the gyroscope
	angle_offset_vertical = -(MMA7361_vertical - MMA7361) * MMA7361_ratio ;  // Normalize the angle to 0~90¡ã that read by the accelerometer by multiplying 0.375.

	if(angle_offset_vertical > 90) angle_offset_vertical = 90;                // Prevent acceleration angle overflow
	if(angle_offset_vertical < -90) angle_offset_vertical = -90;

	QingHua_AngleCalaulate(angle_offset_vertical,Gyro_Now);                  // angle calculated by Tsinghua angle filtering scheme
}

/*
	Angle control output (to motors)
*/
void AngleControlOut(float angle,float angle_dot)
{
	g_fAngleControlOut = (AngleSet-angle) * P_ANGLE  + angle_dot * D_ANGLE ;  // motor output needed for self-balancing
	//P_ANGLE  D_ANGLE  Macro definition PD parameters required for self-balancing
}


