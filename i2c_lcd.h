/*!
 *  \file    i2c_lcd.h
 *  \author  Patrick Taling (not the original author) 
 *  \date    20/02/2017
 *  \version 1.0
 *
 *  \brief   Simple I2C LCD library to write text to a I2C lcd (PCF8574T adress 0x27) with the ATxmega256a3u.
 *
 *  \details The file i2c_lcd.h is the library for a I2C lcd (PCF8574T adress 0x27)
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
      #include <i2c.h>
   \endverbatim
 *           \par
 *
 *           \note An AVR-project can use multiple I2C's. One shoud take care that
 *           in different source files there are no multiple I2C
 *           definitions for the same I2C.
 */

#include <util/delay.h>

#include "i2c.h"

#define LCD_LINES          2             //!< Number of visible lines of the display
#define LCD_DISP_LENGTH    16            //!< Visible characters per line of the display

#if LCD_DISP_LENGTH==16
#define LCD_START_LINE1    0x00          //!< DDRAM address of first char of line 1
#define LCD_START_LINE2    0x40          //!< DDRAM address of first char of line 2
#define LCD_START_LINE3    0x10          //!< DDRAM address of first char of line 3
#define LCD_START_LINE4    0x50          //!< DDRAM address of first char of line 4
#else
#define LCD_START_LINE1    0x00          //!< DDRAM address of first char of line 1
#define LCD_START_LINE2    0x40          //!< DDRAM address of first char of line 2
#define LCD_START_LINE3    0x14          //!< DDRAM address of first char of line 3
#define LCD_START_LINE4    0x54          //!< DDRAM address of first char of line 4
#endif

#define i2c_lcd_RS						(1 << 0)
#define i2c_lcd_RW						(1 << 1)
#define i2c_lcd_E						(1 << 2)
#define i2c_lcd_LED						(1 << 3)
#define i2c_lcd_DB4						(1 << 4)
#define i2c_lcd_DB5						(1 << 5)
#define i2c_lcd_DB6						(1 << 6)
#define i2c_lcd_DB7						(1 << 7)



#define i2c_lcd_rs_lo()					i2c_lcd_status &= ~i2c_lcd_RS
#define i2c_lcd_rs_hi()					i2c_lcd_status |= i2c_lcd_RS

#define i2c_lcd_rw_lo()					i2c_lcd_status &= ~i2c_lcd_RW
#define i2c_lcd_rw_hi()					i2c_lcd_status |= i2c_lcd_RW


#define i2c_lcd_e_lo()					i2c_lcd_status &= ~i2c_lcd_E
#define i2c_lcd_e_hi()					i2c_lcd_status |= i2c_lcd_E

#define i2c_lcd_led_lo()				i2c_lcd_status &= ~i2c_lcd_LED
#define i2c_lcd_led_hi()				i2c_lcd_status |= i2c_lcd_LED


#define HD44780_ENTRY_MODE				0x04
	#define HD44780_EM_SHIFT_CURSOR		0
	#define HD44780_EM_SHIFT_DISPLAY	1
	#define HD44780_EM_DECREMENT		0
	#define HD44780_EM_INCREMENT		2

#define HD44780_DISPLAY_ONOFF			0x08
	#define HD44780_DISPLAY_OFF			0
	#define HD44780_DISPLAY_ON			4
	#define HD44780_CURSOR_OFF			0
	#define HD44780_CURSOR_ON			2
	#define HD44780_CURSOR_NOBLINK		0
	#define HD44780_CURSOR_BLINK		1

#define HD44780_DISPLAY_CURSOR_SHIFT	0x10
	#define HD44780_SHIFT_CURSOR		0
	#define HD44780_SHIFT_DISPLAY		8
	#define HD44780_SHIFT_LEFT			0
	#define HD44780_SHIFT_RIGHT			4

#define HD44780_FUNCTION_SET			0x20
	#define HD44780_FONT5x7				0
	#define HD44780_FONT5x10			4
	#define HD44780_ONE_LINE			0
	#define HD44780_TWO_LINE			8
	#define HD44780_4_BIT				0
	#define HD44780_8_BIT				16

#define HD44780_CGRAM_SET				0x40

#define HD44780_DDRAM_SET				0x80

#define HD44780_CLEAR					0x01

#define i2c_lcd_ADDR					0x27 //I2C LCD adres.

char i2c_lcd_status;

void i2c_lcd_write_status(void);
void i2c_lcd_data_part(char data_part);
void i2c_lcd_write(char data);
void i2c_lcd_write_instruction(char instruction);
void i2c_lcd_write_data(char data);
void i2c_lcd_write_text(char *text);
void i2c_lcd_clear(void);
void i2c_lcd_set_cursor(char x, char y);
void i2c_lcd_led_on(void);
void i2c_lcd_led_off(void);
void i2c_lcd_init(void);
