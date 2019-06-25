#include "common.h"
#include "include.h"
#include "self_include.h"

int g_n1MSEventCount=0;

/********************** Interrupt service codes *******************/

/*
 * self-balancing program's main interrupt service function: PIT0 interrupt service handler
 */
void PIT0_IRQHandler(void)
{
	PIT_Flag_Clear(PIT0);		// Clear interrupt flag
	led_turn(LED0);				// LED0 flicker
	t++;
	g_nSpeedControlPerid++;		// Speed control period is 100, Divided into 20 outputs, 100/20=5
	SpeedControlOutput();		// Speed output periodically
	g_n1MSEventCount++;			// 1ms interrupt count

	/********************** Angle *******************/
	if(g_n1MSEventCount>=3||g_n1MSEventCount==1)
	{
		AD_Calculate();							//AD self-balancing angle, acceleration calculation
		AngleControlOut(g_fCarAngle,Gyro_Now);	// Angle control output //g_fCarAngle final angle, Gyro_Now normalized ENC03 angular velocity
		if(g_n1MSEventCount>=3) g_n1MSEventCount=1;
	}
	/********************** speed *******************/
	else if(g_n1MSEventCount==2)
	{
		g_nSpeedControlCount++;
		if(g_nSpeedControlCount>20)
		{
			g_nSpeedControlCount=0;
			g_nSpeedControlPerid=0;
			SpeedControl();  // speed control
		}
		if(t>3000)
		{
			MotorOutput();
		}
	}
}

/*
    PORTA interrupt service function, camera interrupt service function: PORTA(PTA29 trigger interrupt)
*/
void PORTA_IRQHandler()
{
	uint8  n;    // Pin number
	uint32 flag;

	while(!PORTA_ISFR);
	flag = PORTA_ISFR;
	PORTA_ISFR  = ~0;                                   // Clear interrupt flag

	n = 29;                                             // Field interrupt
	if(flag & (1 << n))                                 // PTA29 trigger interrupt
	{
		camera_vsync();
	}
}

/*
     DMA0 interrupt service function
*/
void DMA0_IRQHandler()
{
	camera_dma();
}


/*!
 *  @brief      PORTE interrupt service function: for nrf interrupt service
 *  @since      v5.0
 */
void PORTE_IRQHandler()
{
	uint8  n;    // Pin number
	uint32 flag;

	flag = PORTE_ISFR;
	PORTE_ISFR  = ~0;                                   // Clear interrupt flag

	n = 27;
	if(flag & (1 << n))                                 // PTE27 trigger interrupt
	{
		nrf_handler();
	}
}

/*
 * speed measuring(encoder) interrupt service PIT1 interrupt service function
 */
void PIT1_IRQHandler(void)
{
	Val_Left  = -ftm_quad_get(FTM1);	// obtain FTM Number of pulses for orthogonal decoding (negative numbers indicate reverse direction)
	Val_Right = ftm_quad_get(FTM2);		// obtain FTM Number of pulses for orthogonal decoding (negative numbers indicate reverse direction)
	ftm_quad_clean(FTM1);
	ftm_quad_clean(FTM2);

	PIT_Flag_Clear(PIT1);				// Clear interrupt flag
}



