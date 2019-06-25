#include "common.h"
#include "include.h"
//#include "self_include.h"

void InterruptService(void)
{
  int i;
  
  g_nSpeedControlPeriod ++;
  SpeedControlOutput();
  
  g_nDirectionControlPeriod ++;
  DirectionControlOutput();
  
  if(g_n1msEventCount>=ControlPeriod)
  {
    g_n1msEventCount=0;
    GetMotorPulse(); // Read two motors' pulse count values
  }
  else if(g_n1msEventCount==1)
  {
    for(i=0;i<Input_Voltage_Average;i++)
      SampleInputVoltage();
  }
  else if(g_n1msEventCount==2)
  {
    GetInputVoltageAverage();
    AngleCalculate();
    AngleControl();
    MotorOutput();
  }
  else if(g_n1msEventCount==3)
  {
    g_nSpeedControlCount++;
    if(g_nSpeedControl>=Speed_Control_Count)
    {
      SpeedControl();
      g_nSpeedControlCount=0;
      g_nSpeedControlPeriod=0;
    }
  }
  else if(g_n1msEventCount=4)
  {
    g_nDirectionControlCount++;
    DirectionVoltageSigma();
    if(g_nDirectionControl>=Direction_Control_Count)
    {
      DirectiondControl();
      g_nDirectionControlCount=0;
      g_nDirectionControlPeriod=0;
    }
  }
}
