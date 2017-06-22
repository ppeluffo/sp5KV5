/*
 * ads7828_sp5KFRTOS.h
 *
 *  Created on: 15/4/2015
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#ifndef SRC_SP5KLIBS_L_ADC7828_H_
#define SRC_SP5KLIBS_L_ADC7828_H_

#include "FRTOS-IO.h"
#include "sp5K_i2c.h"

//#define adc_DEBUG

#ifdef adc_DEBUG
	#define DEBUG_ADC 1
#else
	#define DEBUG_ADC 0
#endif

#define 	ADS7828_ADDR			0x90

#define 	ADS7828_CMD_SD   		0x80	//ADS7828 Single-ended/Differential Select bit.

#define 	ADS7828_CMD_PDMODE0		0x00	//ADS7828 Mode 0: power down
#define 	ADS7828_CMD_PDMODE1 	0x04	//ADS7828 Mode 1: Ref OFF, converter ON
#define 	ADS7828_CMD_PDMODE2   	0x08	//ADS7828 Mode 2: Ref ON, converter OFF
#define 	ADS7828_CMD_PDMODE3   	0x0C	//ADS7828 Mode 3: Ref ON, converter ON.

bool ADC_read(uint8_t channel, uint16_t *value);
bool ADC_readDlgCh(uint8_t channel, uint16_t *adc_value);
bool ADC_test_read( char *s, uint16_t *adcRetValue );

#endif /* SRC_SP5KLIBS_L_ADC7828_H_ */
