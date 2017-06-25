/*
 * l_outputs.h
 *
 *  Created on: 25 de jun. de 2017
 *      Author: pablo
 */

#ifndef SRC_SP5KLIBS_L_OUTPUTS_H_
#define SRC_SP5KLIBS_L_OUTPUTS_H_

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#include <sp5KV5.h>

void OUT_aplicar_consigna_diurna ( void );
void OUT_aplicar_consigna_nocturna ( void );
void OUT_enable_outputs(void);
void OUT_disable_outputs(void);
void OUT0_on(void);
void OUT0_off(void);
void OUT1_on(void);
void OUT1_off(void);
void OUT_pulse(uint8_t channel, uint8_t phase, uint16_t delay);

#endif /* SRC_SP5KLIBS_L_OUTPUTS_H_ */
