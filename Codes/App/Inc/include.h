#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include User-defined header file
 */
#include  "MK60_wdog.h"
#include  "MK60_gpio.h"     //IO port operation
#include  "MK60_uart.h"     //Serial port
#include  "MK60_SysTick.h"
#include  "MK60_lptmr.h"    //Low power timer (delay)
#include  "MK60_i2c.h"      //I2C
#include  "MK60_spi.h"      //SPI
#include  "MK60_ftm.h"      //FTM
#include  "MK60_pit.h"      //PIT
#include  "MK60_rtc.h"      //RTC
#include  "MK60_adc.h"      //ADC
#include  "MK60_dac.h"      //DAC
#include  "MK60_dma.h"      //DMA
#include  "MK60_FLASH.h"    //FLASH
#include  "MK60_can.h"      //CAN
#include  "MK60_sdhc.h"     //SDHC
#include  "MK60_usb.h"      //usb

#include  "VCAN_LED.H"          //LED
#include  "VCAN_KEY.H"          //KEY
#include  "VCAN_MMA7455.h"      //Triaxial acceleration MMA7455
#include  "VCAN_NRF24L0.h"      //Wireless module NRF24L01+
#include  "VCAN_RTC_count.h"    //RTC time conversion
#include  "VCAN_camera.h"       //Camera head file
#include  "VCAN_LCD.h"          //LCD head file
#include  "ff.h"                //FatFs
#include  "VCAN_TSL1401.h"      //Linear CCD
#include  "VCAN_key_event.h"    //Key message processing
#include  "VCAN_NRF24L0_MSG.h"  //Wireless module message processing

#include  "VCAN_BMP.h"          //BMP
#include  "vcan_img2sd.h"       //Store image to a file one SD card
#include  "vcan_sd_app.h"       //SD card application£¨show image firmware on sd card£©

#include  "Vcan_touch.h"        //Touch driver

#include  "VCAN_computer.h"     //Multi-function debugging assistant


#endif  //__INCLUDE_H__
