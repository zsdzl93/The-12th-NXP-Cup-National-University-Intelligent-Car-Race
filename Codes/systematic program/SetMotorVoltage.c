#include "common.h"
#include "include.h"
//#include "self_include.h"

/*PWM output set function*/

viod SetMotorVoltage(float fLeftVoltage,float fRightVoltage)
{
  short nPeriod;
  int nOut;
  
  nPeriod=(short)getReg(PWM_PWMCM);
  
  if(fLeftVoltage>0)
  {
    setReg(PWM_PWMVal1,0);
    nOut=(int)(fLeftVoltage*nPeriod);
    setReg(PWM_PWMVal0,nOut);
  }
  else
  {
    setReg(PWM_PWMVal0,0);
    fLeftVoltage=-fLeftVoltage;
    nOut=(int)(fLeftVoltage*nPeriod);
    setReg(PWM_PWMVal1,nOut);
  }
  
  if(fRightVoltage>0)
  {
    setReg(PWM_PWMVal2,0);
    nOut=(int)(fRightVoltage*nPeriod);
    setReg(PWM_PWMVal3,nOut);
  }
  else
  {
    setReg(PWM_PWMVal3,0);
    fRightVoltage=-fRightVoltage;
    nOut=(int)(fRightVoltage*nPeriod);
    setReg(PWM_PWMVal2,nOut);
  }
  
  Motor_SetLoad;//Reload the PWM value
}



