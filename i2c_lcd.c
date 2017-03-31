/*!
 *  \file    i2c_lcd.c
 *  \author  Patrick Taling (not the original author) 
 *  \date    20/02/2017
 *  \version 1.0
 *
 *  \brief   Simple I2C LCD library to write text to a I2C lcd (PCF8574T adress 0x27) with the ATxmega256a3u.
 *
 *  \details The file i2c_lcd.c is the library for a I2C lcd (PCF8574T adress 0x27)
 
 *	The library needs some parts of the i2c_lcd library from Noel200 from http://www.elektroda.pl/rtvforum/topic2756081.html. 
 *	The library can be downloaded from: http://www.elektroda.pl/rtvforum/login.php?redirect=download.php&id=670533.
 *	Go to LCD_PCF8574T/lcd_pcf/ and use i2c_lcd.c and i2c_lcd.h from the pakkage
 *
 *	The library needs the i2c library from w.e.dolman (<a href="mailto:w.e.dolman@hva.nl">w.e.dolman@hva.nl</a>)
 *	For i2c.c use code 21.8 from "de taal C en de Xmega tweede druk" http://dolman-wim.nl/xmega/book/index.php
 *	For i2c.h use code 21.9 from "de taal C en de Xmega tweede druk" http://dolman-wim.nl/xmega/book/index.php
 *
 *
 * ## Original author information ##

   Obs³uga wyœwietlacza HD44780 po I2C za pomoc¹ PCF8574T.
   2015-01-DASEJ , dasej(at)wp.pl  

   AVR Studio 4.18, programator AVR PROG MKII, 
   Procesor Atmega328P 16 MHz, +5V.

 
 * ####
 *
 *       
 * \verbatim
      #include <i2c_lcd.h>
   \endverbatim
 *           \par
 *
 *           \note An AVR-project can use multiple I2C's. One shoud take care that
 *           in different source files there are no multiple I2C
 *           definitions for the same I2C.
 */

#define F_CPU 2000000UL

#include "i2c_lcd.h"
#include <util/delay.h>

char i2c_lcd_status = 0X00;
volatile uint8_t lcd_line = 0;

void i2c_lcd_write_status(void)
{
	i2c_start(&TWIE, i2c_lcd_ADDR, 0);
	i2c_write(&TWIE, i2c_lcd_status);
	i2c_stop(&TWIE);
}

void i2c_lcd_data_part(char data_part)
{
	i2c_lcd_status &= ~i2c_lcd_DB4 & ~i2c_lcd_DB5 & ~i2c_lcd_DB6 & ~i2c_lcd_DB7;
	if(data_part & 0x01) i2c_lcd_status |= i2c_lcd_DB4;
	if(data_part & 0x02) i2c_lcd_status |= i2c_lcd_DB5;
	if(data_part & 0x04) i2c_lcd_status |= i2c_lcd_DB6;
	if(data_part & 0x08) i2c_lcd_status |= i2c_lcd_DB7;
}

void i2c_lcd_write(char data)
{
	i2c_lcd_e_hi();
	i2c_lcd_data_part(data >> 4);
	i2c_lcd_write_status();
	i2c_lcd_e_lo();
	i2c_lcd_write_status();
	i2c_lcd_e_hi();
	i2c_lcd_data_part(data);
	i2c_lcd_write_status();
	i2c_lcd_e_lo();
	i2c_lcd_write_status();
	_delay_ms(2);
}

void i2c_lcd_write_instruction(char instruction)
{
	i2c_lcd_rw_lo();
	i2c_lcd_rs_lo();
	i2c_lcd_write(instruction);
}

void i2c_lcd_write_data(char data)
{
	i2c_lcd_rs_hi();
	
	switch (data) {
		case '\f':
		i2c_lcd_clear();
		lcd_line = 0;
		break;
		case '\n':
		if (++lcd_line==LCD_LINES) lcd_line = 0;
		i2c_lcd_set_cursor(0, lcd_line);
		break;
		default:
		i2c_lcd_write(data);
		break;
	}
}

void i2c_lcd_write_text(char *text)
{
	while(*text) i2c_lcd_write_data(*text++);
}

void i2c_lcd_clear(void)
{
	lcd_line = 0;
	i2c_lcd_write_instruction(0x01);
}

void i2c_lcd_set_cursor(char x, char y)
{
	uint8_t address;

	#if LCD_LINES==1
	address = LCD_START_LINE1;
	#elif LCD_LINES==2
	if ( y==0 ) {
		address = LCD_START_LINE1;
		} else {
		address = LCD_START_LINE2;
	}
	#else
	if ( y==0 ) {
		address = LCD_START_LINE1;
		} else if ( y==1) {
		address = LCD_START_LINE2;
		} else if ( y==2) {
		address = LCD_START_LINE3;
		} else {
		address = LCD_START_LINE4;
	}
	#endif
	
	i2c_lcd_write_instruction(HD44780_DDRAM_SET | (x + (address * y)));
}

void i2c_lcd_led_on(void)
{
	i2c_lcd_led_hi();
	i2c_lcd_write_status();
}

void i2c_lcd_led_off(void)
{
	i2c_lcd_led_lo();
	i2c_lcd_write_status();
}

void i2c_lcd_home(void)
{
	lcd_line = 0;
	i2c_lcd_set_cursor(0,0);
}

void i2c_lcd_init(void)
{
	char i;
	_delay_ms(15);
	for(i = 0; i < 3; i++)
	{
		i2c_lcd_data_part(0x03);
		i2c_lcd_e_hi();
		i2c_lcd_write_status();
		
		i2c_lcd_e_lo();
		i2c_lcd_write_status();
		_delay_ms(4);
	}
	i2c_lcd_data_part(0x02);
	i2c_lcd_e_hi();
	i2c_lcd_write_status();
	
	i2c_lcd_e_lo();
	i2c_lcd_write_status();
	_delay_ms(1);
	
	i2c_lcd_write_instruction(HD44780_FUNCTION_SET | HD44780_FONT5x10 | HD44780_TWO_LINE | HD44780_4_BIT); // interfejs 4-bity, 2-linie, znak 5x7
	i2c_lcd_write_instruction(HD44780_DISPLAY_ONOFF | HD44780_DISPLAY_OFF); // wy³¹czenie wyswietlacza
	i2c_lcd_write_instruction(HD44780_CLEAR); // czyszczenie zawartosæi pamieci DDRAM
	_delay_ms(2);
	i2c_lcd_write_instruction(HD44780_ENTRY_MODE | HD44780_EM_SHIFT_CURSOR | HD44780_EM_INCREMENT);// inkrementaja adresu i przesuwanie kursora
	i2c_lcd_write_instruction(HD44780_DISPLAY_ONOFF | HD44780_DISPLAY_ON | HD44780_CURSOR_OFF | HD44780_CURSOR_NOBLINK); // w³¹cz LCD, bez kursora i mrugania
}
