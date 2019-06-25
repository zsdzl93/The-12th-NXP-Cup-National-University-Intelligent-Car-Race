#include "common.h"
#include "include.h"
#include "self_include.h"

int Speed_Ki=0;
float g_fCarSpeedOld;
int sum_error=0;
int g_fSpeedControlErrorOld_R=0,g_fSpeedControlErrorOld_L=0;


/*
bangbang
*/

/*
SpeedControl function
*/
void SpeedControl(void)
{
	if (downhillDelay>0) Val_Set=70;
	else if (downhillDelay==0)
		Val_Set=SpeedSet[SpeedLevel];


	g_fSpeedControlErrorOld_L=g_fSpeedControlError_L;
	g_fSpeedControlErrorOld_R=g_fSpeedControlError_R;

	g_fCarSpeed=(Val_Left+Val_Right)/2;	//speed measured = (left speed + right speed)/2 (speed hear is left and right pulse number) -400 to 400


	error=(Val_Set-g_fCarSpeed);
	if (abs(error)>50) Speed_Ki=0;
	else Speed_Ki=1;

	sum_error+=error;
	if(sum_error>500)
	sum_error=500;
	if(sum_error<-500)
	sum_error=-500;

	g_fSpeedControlError_L=(Kp2*error+Kd2*(error-LastError)+Speed_Ki*Ki2*sum_error);//(Kp2*(error-LastError)+Kd2*(error-2*LastError+PrevError)+Speed_Ki*Ki2*error)
	g_fSpeedControlError_R=(Kp2*error+Kd2*(error-LastError)+Speed_Ki*Ki2*sum_error);
	g_fSpeedControlDelta_L=g_fSpeedControlError_L-g_fSpeedControlErrorOld_L;
	g_fSpeedControlDelta_R=g_fSpeedControlError_R-g_fSpeedControlErrorOld_R;
//	PrevError=LastError;
	LastError=error;
  
}

/*
SpeedControlOutput function
*/
void SpeedControlOutput()
{
	g_fSpeedControl_L= g_fSpeedControlDelta_L/SPEED_CONTROL_PERIOD*(g_nSpeedControlPerid+1)+g_fSpeedControlErrorOld_L;    //
	g_fSpeedControl_R= g_fSpeedControlDelta_R/SPEED_CONTROL_PERIOD*(g_nSpeedControlPerid+1)+g_fSpeedControlErrorOld_R;    //

	if(g_fSpeedControl_L>1000)
	{
		g_fSpeedControl_L=1000;
	}
	if(g_fSpeedControl_R>1000)
	{
		g_fSpeedControl_R=1000;
	}
	if(g_fSpeedControl_L<-1000)
	{
		g_fSpeedControl_L=-1000;
	}
	if(g_fSpeedControl_R<-1000)
	{
		g_fSpeedControl_R=-1000;
	}
}
