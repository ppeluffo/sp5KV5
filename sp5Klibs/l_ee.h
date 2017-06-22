/*
 * ee_sp5K.h
 *
 *  Created on: 26/10/2015
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#ifndef SRC_SP5KLIBS_L_EE_H_
#define SRC_SP5KLIBS_L_EE_H_

#include <sp5KV5.h>
#include "FRTOS-IO.h"

//#define ee_DEBUG

#ifdef ee_DEBUG
	#define DEBUG_EE 2
#else
	#define DEBUG_EE 0
#endif
//------------------------------------------------------------------------------------
// Identificacion en el bus I2C
#define EE_ADDR			0xA0
#define EE_PAGESIZE		64

uint8_t EE_read( uint16_t eeAddress, char *data, uint8_t length );
uint8_t EE_write( uint16_t eeAddress, char *data, uint8_t length );
bool EE_test_write(char *s0, char *s1);
bool EE_test_read(char *s0, char *s1, char *s2);

#endif /* SRC_SP5KLIBS_L_EE_H_ */
