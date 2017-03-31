/*!
 *  \file    main.c
 *  \author  Patrick Taling 
 *  \date    31/03/2017
 *  \version 1.0
 *
 *  \brief
 *		Simple I2C SHT2x library to measure relative humidity and temperature and write it to an I2C LCD with the ATxmega256a3u.
 *		Rights are owned by the original authors (Atmel, w.e.dolman, Noel200 (Elektroda) and Sensirion)
 *
 *	\Hardware
 *		# 1602 I2c HD44780 LCD from HobbyElectronica https://www.hobbyelectronica.nl/product/1602-lcd-i2c-blauw-backlight/
 *		# HvA Xmega-bord (ATxmega256a3u with programmer/ USB-RS232-interface ATxmega32a4u)
 *		# 3.3V - 5V levelshifter with two BS170 N-channel MOSFET
 *		# Humidity/ temperature sensor module (EBM016) https://www.elektor.nl/humidity-sensor-module-ebm016
 *
 *	\development
 *		# Atmel Studio 7 (version: 7.0.118)
 *		# OS Version: Microsoft Windows NT 6.2.9200.0 (Platform: Win32NT)
 *		# Atmel Gallary (7.8)
 *
 *  \details 
 *		#The file main.c is the main to write text to a I2C lcd (PCF8574T adress 0x27) with the ATxmega256a3u.
 *		#For the use of the main code you need the i2c library from w.e.dolman (<a href="mailto:w.e.dolman@hva.nl">w.e.dolman@hva.nl</a>)
 *		#For i2c.c use code 21.8 from "de taal C en de Xmega tweede druk" http://dolman-wim.nl/xmega/book/index.php
 *		#For i2c.h use code 21.9 from "de taal C en de Xmega tweede druk" http://dolman-wim.nl/xmega/book/index.php
 *     
 *		#The main code needs some parts of the i2c_lcd library from Noel200 from http://www.elektroda.pl/rtvforum/topic2756081.html. 
 *		#The library can be downloaded from: http://www.elektroda.pl/rtvforum/login.php?redirect=download.php&id=670533.
 *		#Go to LCD_PCF8574T/lcd_pcf/ and use i2c_lcd.c and i2c_lcd.h from the pakkage
 *
 *		#The libraby needs some parts of the the SHT2x library from SENIRION from https://www.sensirion.com/en/products/humidity-sensors/humidity-temperature-sensor-sht2x-digital-i2c-accurate/
 *		#The libraby can be download from: https://www.sensirion.com/en/products/all-documents-of-sensirions-humidity-sensors-for-download/
 *		# Go to SHT2x/ Sample Code SHT21 and download the zip file Sensirion_Humidity_Sensors_SHT21_Sample_Code_C-file
 *		\note	the following files aren't fully used. Most parts are used in other .c of .h files or disabled.
 *			DisplayDip204.C, DisplayDip204.h,
 *			I2C_HAL.c, I2C_HAL.h,
 *			System.c, System.h and
 *			io70f3740.h.
 *		\note for the use of two SHT2X you should use different IOs for SDA or additional hardware such as I2C multiplexer.
 *			The adress of the SHT2x can’t be changed.
 *
 * \verbatim
 *		#include <i2c_lcd.h>
 *		#include <I2C_SHT2x.h>
 *		#include <Typedefs.h>
   \endverbatim
 *           \par
 *
 *           \note An AVR-project can use multiple I2C's. One shoud take care that
 *           in different source files there are no multiple i2c_init
 *           definitions for the same I2C.
 */

#define F_CPU 2000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#define BAUD_100K	100000UL
#include "I2C_SHT2x.h"
#include "Typedefs.h"
#include "i2c_lcd.h"

//==============================================================================
int main()
//==============================================================================
	{ // variables
	uint8_t error = 0;
	uint8_t userRegister;
	int endOfBattery;
	nt16 sRH;
	float humidityRH = 0;
	nt16 sT; 
	float temperatureC = 0; 
	uint8_t SerialNumber_SHT2x[8];
	
	char buffer[20];
	uint16_t h;
	
	i2c_init(&TWIE, TWI_BAUD(F_CPU, BAUD_100K));
	PORTE.DIRSET = PIN1_bm|PIN0_bm; //SDA 0 SCL 1
	
	i2c_lcd_init();
	i2c_lcd_led_on();
	i2c_lcd_set_cursor(0,0);
	
	_delay_ms(15);		//initialization powerUp SHT2x (15ms)

	i2c_lcd_clear();
	_delay_us(1500);	//initialization powerUp LCD (1.5ms)

	i2c_lcd_set_cursor(0,0);
	i2c_lcd_write_text("T :");
	i2c_lcd_write_text("--.-- C");

	i2c_lcd_set_cursor(0,1);
	i2c_lcd_write_text("RH:");
	i2c_lcd_write_text("--.-- %");

	while(1)
	{ 						
		error = 0; // reset error status
		
		//=====================================================
		// --- Reset sensor by command ---
		//=====================================================
		error |= i2c_SHT2x_SoftReset();
		
		//=====================================================
		// --- Read the sensors serial number (64bit) ---
		//=====================================================
		error |= i2c_SHT2x_GetSerialNumber(SerialNumber_SHT2x);
		
		//=====================================================
		// --- Set Resolution e.g. RH 10bit, Temp 13bit ---
		//=====================================================
		error |= i2c_SHT2x_ReadUserRegister(&userRegister);
		userRegister = (userRegister & ~SHT2x_RES_MASK) | SHT2x_RES_10_13BIT;
		error |= i2c_SHT2x_WriteUserRegister(&userRegister);
		
		//=====================================================
		// --- measure humidity with "Hold Master Mode (HM)" ---
		//=====================================================
		error |= i2c_SHT2x_MeasureHM(TEMP, &sT);
		error |= i2c_SHT2x_MeasureHM(HUMIDITY, &sRH);
		
		//=====================================================
		// --- measure temperature with "Polling Mode" (no hold master) ---
		// --- Disabled due to error in TEMP ----
		//=====================================================
		//error |= i2c_SHT2x_MeasurePoll(HUMIDITY, &sRH);		//works fine
		//error |= i2c_SHT2x_MeasurePoll(TEMP, &sT);			// Has an error in reading measuring
		
		//=====================================================
		//-- calculate humidity and temperature --
		//=====================================================
		temperatureC = i2c_SHT2x_CalcTemperatureC(sT.u16);
		humidityRH = i2c_SHT2x_CalcRH(sRH.u16);
		
		//=====================================================
		// --- check end of battery status (eob)---
		//=====================================================
		error |= i2c_SHT2x_ReadUserRegister(&userRegister); //get actual user reg
		if( (userRegister & SHT2x_EOB_MASK) == SHT2x_EOB_ON ) endOfBattery = TRUE;
		else endOfBattery = FALSE;
		
		//=====================================================
		//-- write humidity, temperature or error values on LCD --
		//=====================================================
		if(error != 0){
			i2c_lcd_clear();
			_delay_us(1500);
			i2c_lcd_write_text("An ERROR");
			i2c_lcd_set_cursor(0,1);
			i2c_lcd_write_text("has occurred");
		}
		else if (endOfBattery){
			i2c_lcd_clear();
			_delay_us(1500);
			i2c_lcd_write_text("LOW BATTERY");
		}
		else{
			i2c_lcd_clear();
			_delay_us(1500);
			i2c_lcd_set_cursor(0,0);
			h= temperatureC;
			i2c_lcd_write_text("T :");
			utoa(temperatureC, buffer, 10);
			i2c_lcd_write_text(buffer);
			i2c_lcd_write_text(".");
			utoa((temperatureC-h)*100, buffer, 10);
			i2c_lcd_write_text(buffer);
			i2c_lcd_write_text(" C");
						
			i2c_lcd_set_cursor(0,1);
			h= humidityRH;
			i2c_lcd_write_text("RH:");
			utoa(humidityRH, buffer, 10);
			i2c_lcd_write_text(buffer);
			i2c_lcd_write_text(".");
			utoa((humidityRH-h)*100, buffer, 10);
			i2c_lcd_write_text(buffer);
			i2c_lcd_write_text(" %");
		}
		
		_delay_ms(300); // wait 3s for next measurement
	}
}