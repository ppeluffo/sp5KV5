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
void OUT_pulse( uint8_t channel_id, char fase, uint8_t pulse_width );

void OUT0_disable(void);
void OUT0_on(void);
void OUT0_off(void);

void OUT1_disable(void);
void OUT1_on(void);
void OUT1_off(void);

uint8_t OUTA_0(void);
uint8_t OUTA_1(void);
uint8_t OUTB_0(void);
uint8_t OUTB_1(void);

void OUTPUT_DRV_disable(void);
void OUTPUT_DRV_enable(void);

#define OUT_open_valve_0() ( OUT_pulse( 0, '+', 200 ))
#define OUT_close_valve_0() ( OUT_pulse( 0, '-', 200 ))
#define OUT_open_valve_1() ( OUT_pulse( 1, '+', 200 ))
#define OUT_close_valve_1() ( OUT_pulse( 1, '-', 200 ))


#endif /* SRC_SP5KLIBS_L_OUTPUTS_H_ */
