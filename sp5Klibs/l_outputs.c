/*
 * l_outputs.c
 *
 *  Created on: 25 de jun. de 2017
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#include "l_outputs.h"

// --------------------------------------------------------------------------------
void OUT_aplicar_consigna_diurna ( void )
{
	// Una consigna es la activacion simultanea de 2 valvulas, en las cuales un
	// se abre y la otra se cierra.
	// Cierro la valvula 1
	// Abro la valvula 2
	// Para abrir una valvula debemos poner una fase 10.
	// Para cerrar es 01
	// Para abrir o cerrar una valvula se aplica un pulso de 100ms

	// Cierro la valvula 0
	OUT_close_valve_0();

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	// Abro la valvula 1
	OUT_open_valve_1();

	systemVars.outputs.consigna_aplicada = CONSIGNA_DIURNA;

	// Dejo el sistema de salidas en reposo para que no consuma
	IO_outputs_sleep_on();

}
//------------------------------------------------------------------------------------
void OUT_aplicar_consigna_nocturna ( void )
{
	// Abro la valvula 1
	// Cierro la valvula 2

	// Abro la valvula 0
	OUT_open_valve_0();

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	// Cierro la valvula 1
	OUT_close_valve_1();

	systemVars.outputs.consigna_aplicada = CONSIGNA_DIURNA;

	// Dejo el sistema de salidas en reposo para que no consuma
	IO_outputs_A1ENBL_off();
	IO_outputs_sleep_on();

}
//------------------------------------------------------------------------------------
void OUT0_on(void)
{
	// La salida OUT_0 corresponde al canal A (A1,A2) del DRV8814_1
	// Por norma ponemos la salida AOUT1 en 1 y AOUT2 en 0 ( APHASE = 1 ).

	// OJO: El MCP1 tiene por defecto los pull-ups correspondientes a las salidas deshabilitados
	// para ahorrar corriente. Lo que debo hacer es activar los pullups.
	MCP_write( MCP1_ADDR, MCP1_GPPUA, 0xFF );
	MCP_write( MCP1_ADDR, MCP1_GPPUB, 0xFF );

	IO_outputs_sleep_on();		// Sleep_pin = 1
	IO_outputs_reset_on();		// Reset pin = 1
	IO_outputs_A1PHASE_on(); 	// Phase = 1: AOUT1->H, AOUT2->L
	IO_outputs_A1ENBL_on();		// A1ENABL = 1.

}
//------------------------------------------------------------------------------------
void OUT0_off(void)
{
	// Pongo el driver en reposo.

	IO_outputs_A1ENBL_off();		// A1ENABL = 0. ( deshabilito el bridge )
	IO_outputs_sleep_off();			// Sleep_pin = 0 ( low power )
	IO_outputs_A1ENBL_off();

}
//------------------------------------------------------------------------------------
void OUT1_on(void)
{
	// La salida OUT_1 corresponde al canal B (b1,B2) del DRV8814_1
	// Por norma ponemos la salida BOUT1 en 1 y BOUT2 en 0 ( BPHASE = 1 ).

	// OJO: El MCP1 tiene por defecto los pull-ups correspondientes a las salidas deshabilitados
	// para ahorrar corriente. Lo que debo hacer es activar los pullups.
	MCP_write( MCP1_ADDR, MCP1_GPPUA, 0xFF );
	MCP_write( MCP1_ADDR, MCP1_GPPUB, 0xFF );

	IO_outputs_sleep_on();		// Sleep_pin = 1
	IO_outputs_reset_on();		// Reset pin = 1
	IO_outputs_B1PHASE_off(); 	// Phase = 0: BOUT1->L, BOUT2->H
	IO_outputs_B1ENBL_on();		// A1ENABL = 1.

}
//------------------------------------------------------------------------------------
void OUT1_off(void)
{
	// Pongo el driver en reposo.

	IO_outputs_B1ENBL_off();		// B1ENABL = 0. ( deshabilito el bridge )
	IO_outputs_sleep_off();			// Sleep_pin = 0 ( low power )
	IO_outputs_B1ENBL_off();

}
//------------------------------------------------------------------------------------
void OUT_pulse( uint8_t channel_id, char fase, uint8_t pulse_width )
{
	// channel_id: puede ser 0 ( valvula conectada a la salida 0, A1,A2) o 1 ( B1, B2 )
	// fase: '+' (P0=1,P1=0), '-' (P0=0,P1=1)
	// duracion: ms.de duracion del pulso.

uint8_t regOlatA, regOlatB;

	// Leo y guardo el valor de los pull-ups para restaurarlos al salir.
	MCP_read( MCP1_ADDR, MCP1_GPPUA, &regOlatA );
	MCP_write( MCP1_ADDR, MCP1_GPPUA, 0xFF );
	MCP_read( MCP1_ADDR, MCP1_GPPUB, &regOlatB );
	MCP_write( MCP1_ADDR, MCP1_GPPUB, 0xFF );

	IO_outputs_sleep_on();		// Sleep_pin = 1
	IO_outputs_reset_on();		// Reset pin = 1

	switch(channel_id) {
	case 0:	// Salidas AOUT1,AOUT2
		if ( fase == '+') {
			IO_outputs_A1PHASE_on();
		} else if ( fase == '-') {
			IO_outputs_A1PHASE_off();
		} else {
			return;
		}
		IO_outputs_A1ENBL_on();		// A1ENABL = 1.
		vTaskDelay( ( TickType_t)(pulse_width / portTICK_RATE_MS ) );
		IO_outputs_A1ENBL_off();	// A1ENABL = 0. disable bridge
		break;

	case 1: // Salidas BOUT1, BOUT2
		if ( fase == '+') {
			IO_outputs_B1PHASE_off();
		} else if ( fase == '-') {
			IO_outputs_B1PHASE_on();
		} else {
			return;
		}
		IO_outputs_B1ENBL_on();		// B1ENABL = 1.
		vTaskDelay( ( TickType_t)(pulse_width / portTICK_RATE_MS ) );
		IO_outputs_B1ENBL_off();	// B1ENABL = 0. disable bridge
		break;
	}

	IO_outputs_sleep_off();		// Low power

	// Restauro los drivers
	MCP_write( MCP1_ADDR, MCP1_GPPUA, regOlatA );
	MCP_write( MCP1_ADDR, MCP1_GPPUB, regOlatB );

}
//------------------------------------------------------------------------------------
