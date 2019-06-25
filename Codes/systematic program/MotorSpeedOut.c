#include "common.h"
#include "include.h"
//#include "self_include.h"

/* Add dead zone constant output saturation processing to the output of the left and right electrodes */

void MotorSpeedOut(void)
{
  float fLeftVal,fRightVal;
  
  fLeftVal=g_fLeftMotorOut;
  fRightVal=g_fRightMotorOut;
  
  if(fLeftVal>0)
    fLeftVal+=Motor_Out_DeadVal;
  else if(fLeftVal<0)
    fLeftVal-=Motor_Out_DeadVal;
  
  if(fRightVal>0)
    fRightVal+=Motor_Out_DeadVal;
  else if(fLeftVal<0)
    fRightVal-=Motor_Out_DeadVal;
  
  if(fLeftVal>Motor_Out_Max)
    fLeftVal=Motor_Out_Max;
  if(fLeftVal<Motor_Out_Min)
    fLeftVal=Motor_Out_Min;
  if(fRightVal>Motor_Out_Max)
    fRightVal=Motor_Out_Max;
  if(fRightVal<Motor_Out_Min)
    fRightVal=Motor_Out_Min;
  
  SetMotorVoltage(fLeftVal,fRightVal);
}
