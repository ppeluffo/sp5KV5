/*
 * sp5KV5_defs.h
 *
 *  Created on: 8 de may. de 2018
 *      Author: pablo
 */

#ifndef SRC_SP5KV5_DEFS_H_
#define SRC_SP5KV5_DEFS_H_

#define SP5K_REV "5.2.0"
#define SP5K_DATE "@ 20180523"

#define SP5K_MODELO "sp5KV3 HW:avr1284P R5.0"
#define SP5K_VERSION "FW:FRTOS8"

//#define SP5KV5_8CH
#define SP5KV5_3CH

#ifdef SP5KV5_3CH
	#define NRO_ANALOG_CHANNELS		3
	#define NRO_DIGITAL_CHANNELS 	2
#endif

#ifdef SP5KV5_8CH
	#define NRO_ANALOG_CHANNELS		8
	#define NRO_DIGITAL_CHANNELS 	4
#endif


#endif /* SRC_SP5KV5_DEFS_H_ */
