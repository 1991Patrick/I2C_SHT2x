/*!
 *  \file    main.c
 *  \author  Patrick Taling 
 *  \date    31/03/2017
 *  \version 1.0
 *
 *  \brief
 *		Simple I2C SHT2x library to measure relative humidity and temperature and write it to an I2C LCD with the ATxmega256a3u.
 *		Rights are owned by the original authors (Atmel, w.e.dolman, Noel200 (Elektroda) and Sensirion)
 *		The user is responsible for any damage on any devices
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
 *
 *		# All libraries are included in the folder i2c_SHT2x.
 *		# For i2c.c and i2c.h use folder i2c_Dolman
 *		# For i2c_lcd.c and i2c_lcd.h use folder i2c_lcd_Elektroda.
 *		# For I2C_SHT2x.c, I2C_SHT2x.h and Typedefs.h use folder i2c_SHT2x_Sensirion.
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