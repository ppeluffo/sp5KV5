/*
 * l_ina3221.h
 *
 *  Created on: 13 de oct. de 2017
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_INA3221_H_
#define SRC_SPX_LIBS_L_INA3221_H_

#include "FRTOS-IO.h"
#include "stdio.h"

#include "sp5KV5_defs.h"

//------------------------------------------------------------------------------------
#ifdef SP5KV5_8CH

//#define ina3221_DEBUG

#ifdef ina3221_DEBUG
	#define DEBUG_INA3221 7
#else
	#define DEBUG_INA3221 0
#endif

#define INA3221_ADDR_0			0x80
#define INA3221_ADDR_1			0x82
#define INA3221_ADDR_2			0x86

#define INA3231_CONF_ADDR		0x00
#define INA3221_CH1_SHV			0x01
#define INA3221_CH1_BUSV		0x02
#define INA3221_CH2_SHV			0x03
#define INA3221_CH2_BUSV		0x04
#define INA3221_CH3_SHV			0x05
#define INA3221_CH3_BUSV		0x06
#define INA3221_MFID			0xFE
#define INA3221_DIEID			0xFF

bool INA3221_read( uint8_t devAddress, uint8_t regAddress, char *data, uint8_t length );
bool INA3221_write( uint8_t devAddress, uint8_t regAddress, char *data, uint8_t length );
bool INA3221_test_write(uint8_t id, char *s_regAddr, char *s_value );
uint16_t INA3221_test_read( uint8_t id, uint8_t regAddress );
uint16_t pub_INA3221_read(char *s_inaId, char *s_inaReg);

#endif /* SP5KV5_8CH */

#endif /* SRC_SPX_LIBS_L_INA3221_H_ */
