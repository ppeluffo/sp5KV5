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

static void pv_pulse_valve( uint8_t valve_id, uint8_t pulse_type, uint8_t pulse_width );

typedef enum { V_OPEN, V_CLOSE } t_valve_actions;

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
	pv_pulse_valve(0,V_CLOSE,100);

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	// Abro la valvula 1
	pv_pulse_valve(1,V_OPEN,100);

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
	pv_pulse_valve(0,V_OPEN,100);

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	// Cierro la valvula 1
	pv_pulse_valve(1,V_CLOSE,100);

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
	IO_outputs_B1PHASE_on(); 	// Phase = 1: AOUT1->H, AOUT2->L
	IO_outputs_B1ENBL_on();		// A1ENABL = 1.

}
//------------------------------------------------------------------------------------
void OUT1_off(void)
{
	// Pongo el driver en reposo.

	IO_outputs_B1ENBL_off();		// B1ENABL = 0. ( deshabilito el bridge )
	IO_outputs_sleep_off();			// Sleep_pin = 0 ( low power )

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static void pv_pulse_valve( uint8_t valve_id, uint8_t pulse_type, uint8_t pulse_width )
{
	// valve_id: puese ser 0 ( valvula conectada a la salida 0, A1,A2) o 1 ( B1, B2 )
	// pulse_type: 0-open ( fase = 1 ), 1-close ( fase = 0)
	//

uint8_t regOlatA, regOlatB;

	// Leo y guardo el valor de los pull-ups para restaurarlos al salir.
	MCP_read( MCP1_ADDR, MCP1_GPPUA, &regOlatA );
	MCP_write( MCP1_ADDR, MCP1_GPPUA, 0xFF );
	MCP_read( MCP1_ADDR, MCP1_GPPUB, &regOlatB );
	MCP_write( MCP1_ADDR, MCP1_GPPUB, 0xFF );

	IO_outputs_sleep_on();		// Sleep_pin = 1
	IO_outputs_reset_on();		// Reset pin = 1

	switch(valve_id) {
	case 0:	// Salidas AOUT1,AOUT2
		( pulse_type == V_OPEN ) ? IO_outputs_A1PHASE_on() : IO_outputs_A1PHASE_off();
		IO_outputs_A1ENBL_on();		// A1ENABL = 1.
		vTaskDelay( ( TickType_t)(pulse_width / portTICK_RATE_MS ) );
		IO_outputs_A1ENBL_off();	// A1ENABL = 0. disable bridge
		break;
	case 1: // Salidas BOUT1, BOUT2
		( pulse_type == V_OPEN ) ? IO_outputs_B1PHASE_on() : IO_outputs_B1PHASE_off();
		IO_outputs_B1ENBL_on();		// B1ENABL = 1.
		vTaskDelay( ( TickType_t)(pulse_width / portTICK_RATE_MS ) );
		IO_outputs_B1ENBL_off();	// b1ENABL = 0. disable bridge
		break;
	}

	IO_outputs_sleep_off();		// Low power

	// Restauro los drivers
	MCP_write( MCP1_ADDR, MCP1_GPPUA, regOlatA );
	MCP_write( MCP1_ADDR, MCP1_GPPUB, regOlatB );

}
//------------------------------------------------------------------------------------
