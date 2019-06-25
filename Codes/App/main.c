#include "common.h"
#include "include.h"
#include "self_include.h"
#include "main.h"

/*
   main function
*/

   void main(void)
{
	init_all();
	while(1)
	{
		Switch();
		CameraGet(); //Camera capture information //Choose LCD_ST7735S of USE_LCD in App\inc\MK60_conf.h
		dataAnalysis() ;
		DirectionControl();
		DirectionControlOutput();
	}
}
