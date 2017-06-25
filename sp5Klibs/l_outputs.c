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

//	 MCP_outsPulse( systemVars.consigna.chVA , 1, 100 );	// Cierro la valvula 1
	 vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
//	 MCP_outsPulse( systemVars.consigna.chVB , 0, 100 );	// Abro la valvula 2

	 systemVars.outputs.consigna_aplicada = CONSIGNA_DIURNA;

	 // Dejo el sistema de salidas en reposo para que no consuma
	 IO_outputs_A1ENBL_off();
	 IO_outputs_sleep_on();

}
/*------------------------------------------------------------------------------------*/
void OUT_aplicar_consigna_nocturna ( void )
{
	// Abro la valvula 1
	// Cierro la valvula 2

	// Por las dudas, reconfiguro el MCP
	//pvMCP_init_MCP1(1);

//	 MCP_outsPulse( systemVars.consigna.chVA , 0, 100 );	// Abro la valvula 1
	 vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
//	 MCP_outsPulse( systemVars.consigna.chVB , 1, 100 );	// Cierro la valvula 2

	 systemVars.outputs.consigna_aplicada = CONSIGNA_DIURNA;

	 // Dejo el sistema de salidas en reposo para que no consuma
	 IO_outputs_A1ENBL_off();
	 IO_outputs_sleep_on();

}
//------------------------------------------------------------------------------------
void OUT_enable_outputs(void)
{

}
//------------------------------------------------------------------------------------
void OUT_disable_outputs(void)
{

}
//------------------------------------------------------------------------------------
void OUT0_on(void)
{

}
//------------------------------------------------------------------------------------
void OUT0_off(void)
{

}
//------------------------------------------------------------------------------------
void OUT1_on(void)
{

}
//------------------------------------------------------------------------------------
void OUT1_off(void)
{

}
//------------------------------------------------------------------------------------
void OUT_pulse(uint8_t channel, uint8_t phase, uint16_t delay)
{

}
//------------------------------------------------------------------------------------
