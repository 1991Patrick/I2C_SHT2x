/*!
 *  \file    I2C_SHT2x.h
 *  \author  Patrick Taling (not the original author from the original code of SENSIRION AG) 
 *  \date    31/03/2017
 *  \version 1.0
 *
 *  \brief   Simple I2C SHT2x library to measure relative humidity and temperature and write it to an I2C LCD with the ATxmega256a3u.
 *
 *  \details The file I2C_SHT2x.h is the library for the SHT2x humidity and temperature sensor
 *
 *	The library needs the i2c library from w.e.dolman (<a href="mailto:w.e.dolman@hva.nl">w.e.dolman@hva.nl</a>)
 *	For i2c.c use code 21.8 from "de taal C en de Xmega tweede druk" http://dolman-wim.nl/xmega/book/index.php
 *	For i2c.h use code 21.9 from "de taal C en de Xmega tweede druk" http://dolman-wim.nl/xmega/book/index.php
 *
 *	The libraby needs some parts of the the SHT2x library from SENIRION from https://www.sensirion.com/en/products/humidity-sensors/humidity-temperature-sensor-sht2x-digital-i2c-accurate/
 *	The libraby can be download from: https://www.sensirion.com/en/products/all-documents-of-sensirions-humidity-sensors-for-download/
 *	Go to SHT2x/ Sample Code SHT21 and download the zip file Sensirion_Humidity_Sensors_SHT21_Sample_Code_C-file
 *
 *		\note	the following files aren't fully used. Most parts are used in other .c of .h files or disabled.
 *			DisplayDip204.C, DisplayDip204.h,
 *			I2C_HAL.c, I2C_HAL.h,
 *			System.c, System.h and
 *			io70f3740.h.
 *		\note for the use of two SHT2X you should use different IOs for SDA or additional hardware such as I2C multiplexer.
 *			The adress of the SHT2x can’t be changed.
 *			
 *
 * \ORIGINAL AUTHOR INFORMATION
 * ==============================================================================
 * S E N S I R I O N AG, Laubisruetistr. 50, CH-8712 Staefa, Switzerland
 * ==============================================================================
 * Project : SHT2x Sample Code (V1.2)
 * File : SHT2x.c
 * Author : MST
 * Controller: NEC V850/SG3 (uPD70F3740)
 * Compiler : IAR compiler for V850 (3.50A)
 * Brief : Sensor layer. Functions for sensor access
 * ==============================================================================
 *
 *       
 * \verbatim
      #include <I2C_SHT2x.h>
      #include <i2c.h>
      #include <Typedefs.h>
   \endverbatim
 *           \par
 *
 *           \note An AVR-project can use multiple I2C's. One shoud take care that
 *           in different source files there are no multiple I2C
 *           definitions for the same I2C.
 */

#ifndef SHT2x_H
#define SHT2x_H

#include "i2c.h"
#include "Typedefs.h"

// CRC
#define POLYNOMIAL 0x131; //P(x)=x^8+x^5+x^4+1 = 100110001

#define	TRIG_T_MEASUREMENT_HM  		0xE3 // command trig. temp meas. hold master
#define	TRIG_RH_MEASUREMENT_HM  	0xE5 // command trig. humidity meas. hold master
#define	TRIG_T_MEASUREMENT_POLL  	0xF3 // command trig. temp meas. no hold master
#define	TRIG_RH_MEASUREMENT_POLL  	0xF5 // command trig. humidity meas. no hold master
#define	USER_REG_W  				0xE6 // command writing user register
#define	USER_REG_R 					0xE7 // command reading user register
#define	SOFT_RESET 					0xFE // command soft reset

#define	SHT2x_RES_12_14BIT 			0x00 // RH=12bit, T=14bit
#define	SHT2x_RES_8_12BIT 			0x01 // RH= 8bit, T=12bit
#define	SHT2x_RES_10_13BIT 			0x80 // RH=10bit, T=13bit
#define	SHT2x_RES_11_11BIT			0x81 // RH=11bit, T=11bit
#define	SHT2x_RES_MASK 				0x81 // Mask for res. bits (7,0) in user reg.

#define	SHT2x_EOB_ON 				0x40 // end of battery
#define	SHT2x_EOB_MASK 				0x40 // Mask for EOB bit(6) in user reg.

#define	SHT2x_HEATER_ON 			0x04 // heater on
#define	SHT2x_HEATER_OFF 			0x00 // heater off
#define	SHT2x_HEATER_MASK 			0x04 // Mask for Heater bit(2) in user reg.

#define LOW 						0
#define	HIGH						1

#define TRUE						1
#define FALSE						0

#define CONSTANT_6					6
#define CONSTANT_125				125
#define CONSTANT_46_85				46.85
#define CONSTANT_175_72				175.72
#define RESOLUTION_16				65536 //2^16

#define ACK_ERROR 					0x01
#define	TIME_OUT_ERROR				0x02
#define	CHECKSUM_ERROR 				0x04
#define	UNIT_ERROR 					0x08

#define CMD_CHIP_MEMORY_LOC_1		0xFA
#define ADR_CHIP_MEMORY_LOC_1		0x0F
#define CMD_CHIP_MEMORY_LOC_2		0xFC
#define ADR_CHIP_MEMORY_LOC_2		0xC9

// #define ACK						0 in i2c.h 
// #define NO_ACK					1 in i2c.h

typedef enum{
	HUMIDITY,
	TEMP
}etSHT2xMeasureType;

#define	I2C_SHT2x_ADR  					0x40	// 0x80 -> 128 sensor I2C address + write bit 0x81 -> 129 sensor I2C address + read bit
												// Watch out for bit shifting with the I2C (TWI) protocol in i2c.c
#define	I2C_SHT2x_ADR_W  				0x80
#define	I2C_SHT2x_ADR_R  				0x81		

uint8_t i2c_SHT2x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);
uint8_t i2c_SHT2x_ReadUserRegister(uint8_t *pRegisterValue);
uint8_t i2c_SHT2x_WriteUserRegister(uint8_t *pRegisterValue);
uint8_t i2c_SHT2x_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand);
uint8_t i2c_SHT2x_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand);
uint8_t i2c_SHT2x_SoftReset();
float i2c_SHT2x_CalcRH(uint16_t sRH);
float i2c_SHT2x_CalcTemperatureC(uint16_t sT);
uint8_t i2c_SHT2x_GetSerialNumber(uint8_t SerialNumber[]);
#endif