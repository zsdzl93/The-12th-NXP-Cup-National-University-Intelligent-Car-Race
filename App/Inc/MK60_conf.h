/*!
* @file       MK60_conf.c
 * @brief      K60 Platform configuration function implementation file
 */

#ifndef __MK60_CONF_H__
#define __MK60_CONF_H__

/*
 * Define the platform [defined in the project options]
 */
//#define MK60DZ10
//#define MK60F15

/*
 * Define the LCD model
 */
#define LCD_ILI9341     1       // 3.2 inch LCD(野火/秉火)
#define LCD_ST7735S     2       // 1.44 inch LCD      LCD_ST7735S has the same appearance as LCD_ST7735R but register operation is slightly different
#define LCD_ST7735R     3       // 1.44 inch LCD
#define LCD_ILI9325     4       // 3.2 inch LCD(山外)

#define USE_LCD         LCD_ST7735S             //Select the LCD used

/*
 * Choose whether to output debugging information. If not, Comment the following macro definitions.
 */
#define DEBUG_PRINT

/*
 * Define the crystal clock in MHz
 */
#define EXTAL_IN_MHz            (50)

/*
 * Define PLL overclocking frequency (different platforms have different overclocking configurations)
 *
 * If you are not familiar with the overclocking configuration, you can see the following post:
 * K60 KL26 Main frequency and bus frequency - 智能车资料区 - 山外论坛
 * http://www.vcan123.com/forum.php?mod=viewthread&tid=81&page=1&extra=#pid419
 */
#if defined(MK60DZ10)		//The following is the overclocking configuration of the MK60DZ10
/*
 * 定义 PLL 超频 频率
 */
#define CORE_CLK                PLL180      // Select the provided configuration scheme from PLL_e
                                            		// Bus, flexbus, flash frequency are all integer multiples of the core
#define MAX_BUS_CLK             90         // bus      (bus        >= core/16  )
#define MAX_FLEXBUS_CLK         25          // flex bus (flex bus   >= core/16  )
#define MAX_FLASH_CLK           25          // flash    (flash      >= core/16  )

#elif defined(MK60F15)          //The following is the overclocking configuration of the MK60F15

#define CORE_CLK                PLL200      // Select the provided configuration scheme from PLL_e
                                            // Bus, flexbus, flash frequency are all integer multiples of the core
#define MAX_BUS_CLK             100         // bus      (bus        >= core/16  )
#define MAX_FLEXBUS_CLK         50          // flex bus (flex bus   >= core/16  )
#define MAX_FLASH_CLK           25          // flash    (flash      >= core/16  )

#endif

/*********************   Custom: clock frequency, division factor   ********************/
//If CORE_CLK is PLLUSR , it is in custom mode, the following configuration takes effect
//If it is F15 series： MCG_CLK_MHZ = 50u*(VDIV+16)/(PRDIV+1)/2
//If it is DZ10 series：MCG_CLK_MHZ = 50u/*(VDIV+24)(PRDIV+1)
#define PRDIV             10
#define VDIV              29
#define CORE_DIV          0         //  core = mcg/ ( CORE_DIV  + 1 )
#define BUS_DIV           1         //  bus  = mcg/ ( BUS_DIV   + 1 )
#define FLEX_DIV          9         //  flex = mcg/ ( FLEX_DIV  + 1 )
#define FLASH_DIV         8         //  flash= mcg/ ( FLASH_DIV + 1 )

/*
 * Define the serial output port and serial port information of the printf function
 */
#define VCAN_PORT           UART4
#define VCAN_BAUD           115200

/*
 * Configuring the delay function
 */
#if 0
#include "MK60_DWT.h"                       //May be unstable
#define DELAY()         dwt_delay_ms(500)
#define DELAY_MS(ms)    dwt_delay_ms(ms)
#define DELAY_US(us)    dwt_delay_us(us)
#elif   0
#include "MK60_lptmr.h"
#define     DELAY()         lptmr_delay_ms(500)
#define     DELAY_MS(ms)    lptmr_delay_ms(ms)
#define     DELAY_US(us)    lptmr_delay_us(us)
#elif   0
#include "MK60_pit.h"
#define DELAY()         pit_delay_ms(PIT3,500)
#define DELAY_MS(ms)    pit_delay_ms(PIT3,ms)
#define DELAY_US(us)    pit_delay_us(PIT3,us)
#else
#include "MK60_SysTick.h"
#define DELAY()         systick_delay_ms(500)
#define DELAY_MS(ms)    systick_delay_ms(ms)
#define DELAY_US(us)    systick_delay_us(us)
#endif


/*
 * Configuring assertions and their implementation functions
 */
void assert_failed(char *, int);

#if defined( DEBUG )
#define ASSERT(expr) \
    if (!(expr)) \
        assert_failed(__FILE__, __LINE__)
#else
#define ASSERT(expr)
#endif

/*
 * Configuring debug output functions
 */
#if( defined(DEBUG) && defined(DEBUG_PRINT))
#define DEBUG_PRINTF(FORMAT,...)        do{printf(FORMAT,##__VA_ARGS__);}while(0)	/*Comment out the macro content when you don't need to print the debug information*/
#else
#define DEBUG_PRINTF(FORMAT,...)
#endif

/*
 * Pre-boot detection to prevent reuse of the download port
 */
void start_check();
#ifdef DEBUG
#define SRART_CHECK()       start_check()
#else
#define SRART_CHECK()
#endif


#endif /* __MK60_CONF_H__  */
