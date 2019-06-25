#include "common.h"
#include "include.h"
//#include "self_include.h"

void AngleControl(void)
{
  float fValue;
  
  fValue=(Car_Angle_Set-g_fCarAngle)*Angle_Control_P
         +(Car_Angle_Speed_Set-g_fGyroAngleSpeed)*Angle_Control_D;

  g_fAngleControlOut=fValue;  
}