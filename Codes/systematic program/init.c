#include "common.h"
#include "include.h"
#include "self_include.h"

void init_all()
{
	int i;
	DisableInterrupts;//Prohibit all interruptions

	/*************** Initiate LED *********************/
	led_init (LED0);
	led_init (LED1);
	led_init (LED2);
	led_init (LED3);
	/*************** Initiate adc channel *****************/
	adc_init (ZOUT);          //MMA7361 Z axis
	adc_init (Gyro1);        // ENC03 angular velocity

	/*************** Initiate PWM output *****************/
	ftm_pwm_init(FTM0, FTM_CH3,10000,1000);
	ftm_pwm_init(FTM0, FTM_CH4,10000,1000);
	ftm_pwm_init(FTM0, FTM_CH5,10000,1000);
	ftm_pwm_init(FTM0, FTM_CH6,10000,1000);

	/*************** Enable "enable" port ********************/
	gpio_init(PTD15,GPO,1);      // motor "enable" port
	gpio_init(PTA19,GPO,1);      // Gyro G-sel
  
	/*************** Initiate DIP switch *****************/
	gpio_init(PTE0,GPI,0);                         // DIP switch
	port_init_NoALT(PTE0,PULLUP);

	gpio_init(PTE1,GPI,0);                         // DIP switch
	port_init_NoALT(PTE1,PULLUP);

	gpio_init(PTE2,GPI,0);                         // DIP switch
	port_init_NoALT(PTE2,PULLUP);

	gpio_init(PTE3,GPI,0);                         // DIP switch
	port_init_NoALT(PTE3,PULLUP);

	gpio_init(PTE4,GPI,0);                         // DIP switch
	port_init_NoALT(PTE4,PULLUP);

	gpio_init(PTE5,GPI,0);                         // DIP switch
	port_init_NoALT(PTE5,PULLUP);

	gpio_init(PTE6,GPI,0);                         // DIP switch
	port_init_NoALT(PTE6,PULLUP);

	gpio_init(PTE7,GPI,0);                         // DIP switch
	port_init_NoALT(PTE7,PULLUP);
    
	/*************** Initiate NRF *****************/
	nrf_init();

	/*************** Initiate LCD *****************/
	LCD_init();

	/*************** Initiate camera *****************/
	camera_init(imgbuff);

	/***************Initiate FTM1 and FTM2 Orthogonal decoding *****************/
	ftm_quad_init(FTM1);                                    //FTM1 Quadrature decoding initialization(Pins used can be checked in port_cfg.h)
	ftm_quad_init(FTM2);                                    //FTM2 Quadrature decoding initialization(Pins used can be checked in port_cfg.h)

	/*************** Initialize various interrupts ****************/
	/*
	* Self-balancing main program interrupt
	*/
	pit_init_ms(PIT0, 1);                                	// initiate PIT0, set timer: 5ms
	set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);    // Set PIT0's Interrupt reset function: PIT0_IRQHandler
	enable_irq (PIT0_IRQn);                               // Enble PIT0 interrupt

	/*
	* nrf interrupt
	*/
	set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);    			// set PORTE Interrupt service function: PORTE_VECTORn
	enable_irq(PORTE_IRQn);

	/*
	*For encoder speed measurement: FTM1 and FTM2 interrupt
	*/
	pit_init_ms(PIT1, 10);                                 // Initiate PIT1, set timer: 10ms
	set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);     // Set PIT1's Interrupt reset function: PIT1_IRQHandler
	enable_irq (PIT1_IRQn);                                // Enble PIT0 interrupt

	/*
	* For camera field interrupt
	*/
	set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   // Set PITA's Interrupt reset function: PITA_IRQHandler
	/*
	* For DMA interrupt
	*/
	set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     // Set DMA0's Interrupt reset function: DMA0_IRQHandler


	g_fSpeedControl_L=0;
	g_fSpeedControl_R=0;

	/*************** Initialize gryo reading (eliminate the effects of temperature drift) ****************/
	DELAY_MS(1000);							// Delay 1000ms
	float ENC=0;
	for( i=1;i<=1030;i++ )					// Multiple measurement
	{
		ENC+= adc_once(Gyro1,ADC_12bit);	// gyro1
		if( i<=30 )
		{
			ENC=0;
		}
	}

	GYRO_VAL = ( ENC/1000.0 );				// Finding average by multiple measurement

	EnableInterrupts; // Enable interrupts
}

