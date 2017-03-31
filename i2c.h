/*!
 *  \file    i2c.c
 *  \author  Wim Dolman (<a href="mailto:w.e.dolman@hva.nl">w.e.dolman@hva.nl</a>)
 *  \date    11-12-2015
 *  \version 1.0
 *
 *  \brief   Simple I2C function for ATxmega256a3u
 *
 *  \details The files i2c.c and i2c.h are from w.e.dolman (<a href="mailto:w.e.dolman@hva.nl">w.e.dolman@hva.nl</a>)
 *	For i2c.c code 21.8 is used from: "de taal C en de Xmega tweede druk" http://dolman-wim.nl/xmega/book/index.php
 *	For i2c.h code 21.9 is used from: "de taal C en de Xmega tweede druk" http://dolman-wim.nl/xmega/book/index.php
 *       
 * \verbatim
      #include <i2c.h>
   \endverbatim
 *           \par
 *
 *           \note An AVR-project can use multiple I2C's. One shoud take care that
 *           in different source files there are no multiple i2c_init
 *           definitions for the same I2C.
 */

#include <avr/io.h>

#define TWI_BAUD(F_SYS, F_TWI)   ((F_SYS / (2 * F_TWI)) - 5)

#define I2C_ACK     0
#define I2C_NACK    1
#define I2C_READ    1
#define I2C_WRITE   0

#define I2C_STATUS_OK      0
#define I2C_STATUS_BUSY    1
#define I2C_STATUS_NO_ACK  2

void    i2c_init(TWI_t *twi, uint8_t baudRateRegisterSetting);
uint8_t i2c_start(TWI_t *twi, uint8_t address, uint8_t rw);
uint8_t i2c_restart(TWI_t *twi, uint8_t address, uint8_t rw);
void    i2c_stop(TWI_t *twi);
uint8_t i2c_write(TWI_t *twi, uint8_t data);
uint8_t i2c_read(TWI_t *twi, uint8_t ack);