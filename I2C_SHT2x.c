/*!
 *  \file    I2C_SHT2x.c
 *  \author  Patrick Taling (not the original author from the original code of SENSIRION AG) 
 *  \date    31/03/2017
 *  \version 1.0
 *
 *  \brief   Simple I2C SHT2x library to measure relative humidity and temperature and write it to an I2C LCD with the ATxmega256a3u.
 *
 *  \details The file I2C_SHT2x.c is the library for the SHT2x humidity and temperature sensor
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

#define F_CPU 2000000UL
#include "I2C_SHT2x.h"
#include "i2c.h"
#include "Typedefs.h"
#include <util/delay.h>
#include <assert.h>
#include <math.h>


//==============================================================================
uint8_t i2c_SHT2x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
//==============================================================================
{
	uint8_t crc = 0;
	uint8_t byteCtr;
	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
	{ 	crc ^= (data[byteCtr]);
		for (uint8_t bit = 8; bit > 0; --bit)
		{ 	if (crc & 0x80) {crc = (crc << 1) ^ POLYNOMIAL;}
			else crc = (crc << 1);
		}
	}
	if (crc != checksum) return CHECKSUM_ERROR;
	else return 0;
}
//===========================================================================
uint8_t i2c_SHT2x_ReadUserRegister(uint8_t *pRegisterValue)
//===========================================================================
{
	uint8_t checksum; 
	uint8_t error=0; 
	error |= i2c_start(&TWIE, I2C_SHT2x_ADR, 0);
	error |= i2c_write(&TWIE, USER_REG_R);
	i2c_stop(&TWIE);
	error |= i2c_start(&TWIE, I2C_SHT2x_ADR, 1);
	*pRegisterValue = i2c_read(&TWIE, I2C_ACK);
	checksum=i2c_read(&TWIE, I2C_NACK);
	error |= i2c_SHT2x_CheckCrc (pRegisterValue,1,checksum);
	i2c_stop(&TWIE);
	return error;
}
//===========================================================================
uint8_t i2c_SHT2x_WriteUserRegister(uint8_t *pRegisterValue)
//===========================================================================
{
	uint8_t error=0;
	_delay_ms(2000);
	error |= i2c_start(&TWIE, I2C_SHT2x_ADR, 0);
	error |= i2c_write(&TWIE, USER_REG_W);
	error |= i2c_write(&TWIE, *pRegisterValue);
	i2c_stop(&TWIE);
	return error;
}
//===========================================================================
uint8_t i2c_SHT2x_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand)
//===========================================================================
{
	
	uint8_t checksum;
	uint8_t data[2]; 
	uint8_t error=0;
	
	//-- write I2C sensor address and command --
	error |= i2c_start(&TWIE, I2C_SHT2x_ADR, 0);
	
	switch(eSHT2xMeasureType)
	{ 	case HUMIDITY:	error |= i2c_write(&TWIE, TRIG_RH_MEASUREMENT_HM); break;
		case TEMP:		error |= i2c_write(&TWIE, TRIG_T_MEASUREMENT_HM); break;
		default:		assert(0);
	}
	i2c_stop(&TWIE);
	
	//-- wait until hold master is released --
	error |= i2c_start(&TWIE, I2C_SHT2x_ADR, 1);
	
	//-- read two data bytes and one checksum byte --
	pMeasurand->s16.u8H = data[0] = i2c_read(&TWIE, I2C_ACK);
	pMeasurand->s16.u8L = data[1] = i2c_read(&TWIE, I2C_ACK);
	checksum= i2c_read(&TWIE, I2C_NACK);
	
	//-- verify checksum --
	error |= i2c_SHT2x_CheckCrc (data,2,checksum);
	i2c_stop(&TWIE);
	return error;
}
//===========================================================================
uint8_t i2c_SHT2x_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand)
//--- Disabled due to error in TEMP ---	
//--- measuring TEMP doesn't work after the do while loop ---
//===========================================================================
{
	uint8_t checksum; 
	uint8_t data[2];
	uint8_t error=0;
	uint16_t i=0;
	
	//-- write I2C sensor address and command --
	error |= i2c_start(&TWIE, I2C_SHT2x_ADR, 0); // I2C Adr
	switch(eSHT2xMeasureType)
	{ 	case HUMIDITY:	error |= i2c_write(&TWIE, TRIG_RH_MEASUREMENT_POLL); break;
		case TEMP:		error |= i2c_write(&TWIE, TRIG_T_MEASUREMENT_POLL); break;
		default:		assert(0);
	}
	_delay_us(20);
	i2c_stop(&TWIE);
			
	//-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
	do
		{
		_delay_us(10000); //delay 10ms			Max delay is  0.2s (20*10ms = 200ms)
		if(i++ >= 20) break;
	} while(i2c_start(&TWIE, I2C_SHT2x_ADR, 1) == ACK_ERROR);
	if (i>=20) error |= TIME_OUT_ERROR;
	
	//-- read two data bytes and one checksum byte --
	pMeasurand->s16.u8H = data[0] = i2c_read(&TWIE, I2C_ACK);
	pMeasurand->s16.u8L = data[1] = i2c_read(&TWIE, I2C_ACK);
	checksum= i2c_read(&TWIE, I2C_NACK);
	
	//-- verify checksum --
	error |= i2c_SHT2x_CheckCrc (data,2,checksum);
	i2c_stop(&TWIE);
	return error;
}
//===========================================================================
uint8_t i2c_SHT2x_SoftReset()
//===========================================================================
{
	uint8_t error=0;
	error |= i2c_start(&TWIE, I2C_SHT2x_ADR, 0);
	error |= i2c_write(&TWIE, SOFT_RESET);
	i2c_stop(&TWIE);
	_delay_ms(15);
	return error;
}
//==============================================================================
float i2c_SHT2x_CalcRH(uint16_t sRH)
// --- The original formula from SENSIRION is wrong! ---
//==============================================================================
{
	float humidityRH; 
	sRH &= ~0x0003;
	
	//-- calculate relative humidity [%RH] --
	humidityRH = (float)sRH / RESOLUTION_16;	//RH = -6 + 125 * sRH/2^16;
	humidityRH = humidityRH * CONSTANT_125;
	humidityRH = humidityRH - CONSTANT_6;
	
	return humidityRH;
}
//==============================================================================
float i2c_SHT2x_CalcTemperatureC(uint16_t sT)
// --- The original formula from SENSIRION is wrong! ---
//==============================================================================
{
	float temperatureC;
	sT &= ~0x0003;
	
	//-- calculate temperature [°C] --
	temperatureC = (float)sT / RESOLUTION_16;	//T= -46.85 + 175.72 * ST/2^16
	temperatureC = temperatureC * CONSTANT_175_72;
	temperatureC = temperatureC - CONSTANT_46_85;
	
	return temperatureC;
}

//==============================================================================
uint8_t i2c_SHT2x_GetSerialNumber(uint8_t SerialNumber[]) 
//==============================================================================
{
	uint8_t error=0; //error variable
	//Read from memory location 1
	error |= i2c_start(&TWIE, I2C_SHT2x_ADR, 0); //I2C address
	error |= i2c_write(&TWIE, CMD_CHIP_MEMORY_LOC_1); //Command for readout on-chip memory
	error |= i2c_write(&TWIE, ADR_CHIP_MEMORY_LOC_1); //on-chip memory address
	i2c_stop(&TWIE);
	
	error |= i2c_start(&TWIE, I2C_SHT2x_ADR, 1); //I2C read address
	SerialNumber[5] = i2c_read(&TWIE, I2C_ACK); //Read SNB_3
	i2c_read(&TWIE, I2C_ACK); //Read CRC SNB_3 (CRC is not analyzed)
	SerialNumber[4] = i2c_read(&TWIE, I2C_ACK); //Read SNB_2
	i2c_read(&TWIE, I2C_ACK); //Read CRC SNB_2 (CRC is not analyzed)
	SerialNumber[3] = i2c_read(&TWIE, I2C_ACK); //Read SNB_1
	i2c_read(&TWIE, I2C_ACK);; //Read CRC SNB_1 (CRC is not analyzed)
	SerialNumber[2] = i2c_read(&TWIE, I2C_ACK); //Read SNB_0
	i2c_read(&TWIE, I2C_NACK); //Read CRC SNB_0 (CRC is not analyzed)
	i2c_stop(&TWIE);
	
	//Read from memory location 2
	error |= i2c_start(&TWIE, I2C_SHT2x_ADR, 0); //I2C address
	error |= i2c_write(&TWIE, CMD_CHIP_MEMORY_LOC_2); //Command for readout on-chip memory
	error |= i2c_write(&TWIE, ADR_CHIP_MEMORY_LOC_2); //on-chip memory address
	i2c_stop(&TWIE);

	error |= i2c_start(&TWIE, I2C_SHT2x_ADR, 1); //I2C address
	SerialNumber[1] = i2c_read(&TWIE, I2C_ACK); //Read SNC_1
	SerialNumber[0] = i2c_read(&TWIE, I2C_ACK); //Read SNC_0
	i2c_read(&TWIE, I2C_ACK); //Read CRC SNC0/1 (CRC is not analyzed)
	SerialNumber[7] = i2c_read(&TWIE, I2C_ACK); //Read SNA_1
	SerialNumber[6] = i2c_read(&TWIE, I2C_ACK); //Read SNA_0
	i2c_read(&TWIE, I2C_NACK); //Read CRC SNA0/1 (CRC is not analyzed)
	i2c_stop(&TWIE);
	return error;
}