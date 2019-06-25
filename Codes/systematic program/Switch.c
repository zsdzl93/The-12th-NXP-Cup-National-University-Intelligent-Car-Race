#include "common.h"
#include "include.h"
#include "self_include.h"

/******************** DIP switch ***************************/
void Switch()
{
	DIPcheck();
	DIPprocess();
}

/********************** Check DIP switch ************************/
void DIPcheck()
{
    DIPswitch_1=gpio_get(PTE0); // 1
    DIPswitch_2=gpio_get(PTE1); // 2
    DIPswitch_3=gpio_get(PTE2); // 3
    DIPswitch_4=gpio_get(PTE3); // 4
    DIPswitch_5=gpio_get(PTE4); // Hang up
    DIPswitch_6=gpio_get(PTE5); // 1
    DIPswitch_7=gpio_get(PTE6); // 2
    DIPswitch_8=gpio_get(PTE7); // 3
}

/********************** Process DIP switch ************************/
void DIPprocess()
{
    //Dip switch1 ,2 ,3£¬4  speed control and Dip switch   +DIPswitch_3*2+DIPswitch_4
    Kp3_Level=DIPswitch_1*2+DIPswitch_2;
    Kp3=Kp3_Set[Kp3_Level];
    
    SpeedLevel=DIPswitch_3*4+DIPswitch_4*2+DIPswitch_6;
    Val_Set=SpeedSet[SpeedLevel];
      
    
   // Dip switch6        enter ring from left or right
      if (DIPswitch_7==0)
     {
       ringEnter_R=-1;   // up - right
     } 
    else
    {
        ringEnter_R=1;   // down - left
    }
    
//	//Dip switch7     control uphill and downhill
//	if (DIPswitch_7==0) rampSet=1;
//
//	// Dip switch7     control host computer
//	if (DIPswitch_7==0) SendBox();
//
//	// Dip switch8     draw important lines
//	if (DIPswitch_8==0) LCDdisplay();
}
 
void LCDdisplay()
{
	int i,j;
	for(i=0;i<60;i++)  // middle line
	{
		Site_t dian={ zhong[i],i };
		LCD_point(dian,  BLUE);
	}

	for(j=40,i=0;i<60;i++)// 40 row middle line
	{
		Site_t dian1={ j,i };
		LCD_point(dian1,  GREEN);
	}

	for(i=40,j=0;j<80;j++)// control row
	{
		Site_t dian2={ j,i };
		LCD_point(dian2,  GREEN);
	}

	for(i=40,j=leftRamp[i];i<55;i++)// ramp left line
	{
		Site_t dian2={ leftRamp[i],i };
		LCD_point(dian2, RED);
	}

	for(i=40,j=rightRamp[i];i<55;i++)// ramp right line
	{
		Site_t dian2={ rightRamp[i],i };
		LCD_point(dian2, RED);
	}
//	Site_t site2   = {0, 65};
//	LCD_num( site2 ,Val_Set , BLACK, WHITE);  // display outBoundRow
//
//	Site_t site3   = {0, 90};
//	LCD_num( site3 ,Kp3*10 , BLACK, WHITE);  // display speed Val_Set
//
//	Site_t site4   = {90,0};
//	LCD_num( site4 ,GYRO_VAL , BLACK, WHITE);  // display outBoundRowAverage
//
//	Site_t site5   = {90, 20};
//	LCD_num( site5 , ENC03 , BLACK, WHITE);  // display Middle line error
//
//	Site_t site6   = {90, 40};
//	LCD_num( site6 , GYRO_SET , BLACK, WHITE);  // diaplay
//
//	Site_t site7   = {90, 60};
//	LCD_num( site7 , leftEmptyflag, BLACK, WHITE);  // diaplay
//
//	Site_t site8   = {90, 80};
//	LCD_num( site8 , rightEmptyflag, BLACK, WHITE);  // diaplay
}
