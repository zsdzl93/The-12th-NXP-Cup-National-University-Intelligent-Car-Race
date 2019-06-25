#include "common.h"
#include "include.h"
#include "self_include.h"


/**********************************************************************/
/*	Direction Control
/*	Determine the direction control by PD parameter and middle line.
***********************************************************************/
void DirectionControl()
{
	MiddlelineError = 40 - AverageMiddleline;//average_middleline;
  
	if(obstacleflag==1)
	{
		if (MiddlelineError<0) // turn right
		{
			Kp1=0.028;//0.5 0.08; // 0.5 0.4
			Kd1=0.4;//0.25
		}
		else   //turn left
		{
			Kp1=0.035;// 0.52 0.082;// 0.52 0.42
			Kd1=0.4;//0.25
		}
	}
	else
	{
		if (MiddlelineError<0) // turn right
		{
			if( abs(MiddlelineError)<=Curve)
			{
				Kp1=0.0240;//+ (4.5-4)*(float) MiddlelineError* MiddlelineError/Curve/Curve;    //0.024
				Kd1=0.2;    //0.2
			}
			else //if( abs( MiddlelineError<=(Curve+8) ) )
			{
				Kp1=0.024;//+(4.5-4.3)* (float)MiddlelineError* MiddlelineError/1600;//(Curve+8)/(Curve+8);    //0.024
				Kd1=0.2;
			}
		}
		else // turn left
		{
			if( abs(MiddlelineError)<=Curve)
			{
				Kp1=0.025;//+ (4.5-4)*(float) MiddlelineError* MiddlelineError/Curve/Curve;    //0.022
				Kd1=0.2;//0.2
			}
			else //if( abs( MiddlelineError<=(Curve+8) ) )
			{
				Kp1=0.025;//+(4.5-4.3)* (float)MiddlelineError* MiddlelineError/1600;//(Curve+8)/(Curve+8);    // 0.022
				Kd1=0.2;  //0.2
			}
		}
	}
	/* Compute PID on the center line error and output the control value */
	g_fDirectionControlNew = Kp1*MiddlelineError* g_fCarSpeed * pow( fabs( g_fCarSpeed ), 0.050 )
							+ (MiddlelineError-MiddlelinePreError)*Kd1*g_fCarSpeed ;//Kd1 * 40 * pow( fabs(MiddlelineError/40),0.2 ) ;
  
 
	/* Output limiter for direction control */
	if(stopDelay>0)
	{
		if(g_fDirectionControlNew>5) g_fDirectionControlNew =5;
		if(g_fDirectionControlNew<-5) g_fDirectionControlNew =-5;
	}

	/* Update midline deviation */
	MiddlelinePreError = MiddlelineLastError;
	MiddlelineLastError= MiddlelineError;
}

/******************************************************/
/*      Direction Control Output
/*      directly output (No cycle, no accumulation)
******************************************************/
void DirectionControlOutput()
{
	if (ringflag==1)
	{
		g_fDirectionControlOut = ringEnter_R * Kp3 * g_fCarSpeed * pow( fabs( g_fCarSpeed ), 0.030 );  // Directly output the PID control value //0.050
	}
	else
	{
		g_fDirectionControlOut = g_fDirectionControlNew;
	}
}
