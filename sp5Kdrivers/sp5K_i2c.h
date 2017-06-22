/*
 * sp5K_i2c.h
 *
 *  Created on: 18/10/2015
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 DRIVER
// --------------------------------------------------------------------------------

#ifndef SRC_SP5KDRIVERS_SP5K_I2C_H_
#define SRC_SP5KDRIVERS_SP5K_I2C_H_

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>

#include "FreeRTOS.h"

#define SCL		0
#define SDA		1
#define I2C_MAXTRIES	5

#define ACK 1
#define NACK 0

void i2c_init(void);
bool I2C_masterWrite ( const uint8_t devAddress, const uint8_t devAddressLength, const uint16_t byteAddress, char *pvBuffer, size_t xBytes );
bool I2C_masterRead  ( const uint8_t devAddress, const uint8_t devAddressLength, const uint16_t byteAddress, char *pvBuffer, size_t xBytes );

#endif /* SRC_SP5KDRIVERS_SP5K_I2C_H_ */
