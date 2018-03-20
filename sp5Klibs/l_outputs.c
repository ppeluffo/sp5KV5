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
void OUTPUT_DRV_disable(void)
{
	// Retorno los pull-ups del MCP23018 a su estado normal para que no consuman
	// Ver los valores al inicializarlos en l_mcp.c

	MCP_write( MCP1_ADDR, MCP1_GPPUA, 0x00 );
	MCP_write( MCP1_ADDR, MCP1_GPPUB, 0xF3 );

	IO_outputs_sleep(LOW);
}
// --------------------------------------------------------------------------------
void OUTPUT_DRV_enable(void)
{
	// Debo habilitar los pull-ups del MCP23018 para que pueda drivear los pines
	// Esto consume corriente
	MCP_write( MCP1_ADDR, MCP1_GPPUA, 0xFF );
	MCP_write( MCP1_ADDR, MCP1_GPPUB, 0xFF );

	IO_outputs_sleep(HIGH);
	IO_outputs_reset(HIGH);
}
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

	OUTPUT_DRV_enable();

	// Cierro la valvula 0
	OUT_close_valve_0();

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	// Abro la valvula 1
	OUT_open_valve_1();

	OUTPUT_DRV_disable();
}
//------------------------------------------------------------------------------------
void OUT_aplicar_consigna_nocturna ( void )
{
	// Abro la valvula 1
	// Cierro la valvula 2

	OUTPUT_DRV_enable();
	// Abro la valvula 0
	OUT_open_valve_0();

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	// Cierro la valvula 1
	OUT_close_valve_1();

	OUTPUT_DRV_disable();
}
//------------------------------------------------------------------------------------
void OUT0_disable(void)
{
	// Solo deshabilita la linea de la OUT0.
	IO_outputs_A1ENBL(LOW);

	// Por costumbre ponemos la fase en 0.
	IO_outputs_A1PHASE(LOW);
}
//------------------------------------------------------------------------------------
void OUT0_on(void)
{
	// La salida OUT_0 corresponde al canal A (A1,A2) del DRV8814_1
	// Por norma ponemos la salida AOUT1 en 1 y AOUT2 en 0 ( APHASE = 1 ).
	// Debemos habilitar antes que nada la linea del OUT0.
	// De la Table 2. H-Bridge Logic,

	IO_outputs_A1PHASE(HIGH); 	// Phase = 1: AOUT1->H, AOUT2->L
	IO_outputs_A1ENBL(HIGH);	// A1ENABL = 1.

}
//------------------------------------------------------------------------------------
void OUT0_off(void)
{
	// Pongo la salida en 00.
	// De la Table 2. H-Bridge Logic, para tener Low - Low:

	IO_outputs_A1ENBL(LOW);		// A1ENABL = 0. ( deshabilito el bridge )

	// Por costumbre ponemos la fase en 0.
	IO_outputs_A1PHASE(LOW);

}
//------------------------------------------------------------------------------------
void OUT1_disable(void)
{
	// Solo deshabilita la linea de la OUT1.
	IO_outputs_B1ENBL(LOW);

	// Por costumbre ponemos la fase en 0.
	IO_outputs_B1PHASE(LOW);
}
//------------------------------------------------------------------------------------
void OUT1_on(void)
{
	// La salida OUT_1 corresponde al canal B (b1,B2) del DRV8814_1
	// Por norma ponemos la salida BOUT1 en 1 y BOUT2 en 0 ( BPHASE = 1 ).
	// Cambia la fase porque tenemos los pines cambiados.

	IO_outputs_B1PHASE(LOW); 	// Phase = 0: BOUT1->L, BOUT2->H
	IO_outputs_B1ENBL(HIGH);		// A1ENABL = 1.

}
//------------------------------------------------------------------------------------
void OUT1_off(void)
{
	// De la Table 2. H-Bridge Logic, para tener Low - Low:

	IO_outputs_B1ENBL(LOW);		// B1ENABL = 0. ( deshabilito el bridge )

	// Por costumbre ponemos la fase en 0.
	IO_outputs_B1PHASE(LOW);
}
//------------------------------------------------------------------------------------
void OUT_pulse( uint8_t channel_id, char fase, uint8_t pulse_width )
{
	// channel_id: puede ser 0 ( valvula conectada a la salida 0, A1,A2) o 1 ( B1, B2 )
	// fase: '+' (P0=1,P1=0), '-' (P0=0,P1=1)
	// duracion: ms.de duracion del pulso.

	switch(channel_id) {
	case 0:	// Salidas AOUT1,AOUT2
		if ( fase == '+') {
			IO_outputs_A1PHASE(HIGH);
		} else if ( fase == '-') {
			IO_outputs_A1PHASE(LOW);
		} else {
			break;
		}
		IO_outputs_A1ENBL(HIGH);		// A1ENABL = 1.
		vTaskDelay( ( TickType_t)(pulse_width / portTICK_RATE_MS ) );
		IO_outputs_A1ENBL(LOW);	// A1ENABL = 0. disable bridge
		break;

	case 1: // Salidas BOUT1, BOUT2
		if ( fase == '+') {
			IO_outputs_B1PHASE(LOW);
		} else if ( fase == '-') {
			IO_outputs_B1PHASE(HIGH);
		} else {
			break;
		}
		IO_outputs_B1ENBL(HIGH);		// B1ENABL = 1.
		vTaskDelay( ( TickType_t)(pulse_width / portTICK_RATE_MS ) );
		IO_outputs_B1ENBL(LOW);	// B1ENABL = 0. disable bridge
		break;
	}

	//
	IO_outputs_A1PHASE(LOW);
	IO_outputs_B1PHASE(LOW);
	IO_outputs_A1ENBL(LOW);
	IO_outputs_B1ENBL(LOW);
}
//------------------------------------------------------------------------------------
uint8_t OUTA_1(void)
{
	// La salida OUT_0 corresponde al canal A (A1,A2) del DRV8814_1
	// Por norma ponemos la salida AOUT1 en 1 y AOUT2 en 0 ( APHASE = 1 ).
	// Debemos habilitar antes que nada la linea del OUT0.
	// De la Table 2. H-Bridge Logic,

uint8_t pin;

	OUTPUT_DRV_enable();
	IO_outputs_A1PHASE(HIGH); 	// Phase = 1: AOUT1->H, AOUT2->L
	IO_outputs_sleep(HIGH);
	IO_outputs_reset(HIGH);
	IO_outputs_A1ENBL(HIGH);	// A1ENABL = 1.

	IO_read_fault1(&pin);
	return(pin);

}
//------------------------------------------------------------------------------------
uint8_t OUTA_0(void)
{

uint8_t pin;

	OUTPUT_DRV_enable();
	IO_outputs_A1PHASE(LOW); 	// Phase = 1: AOUT1->H, AOUT2->L
	IO_outputs_sleep(HIGH);
	IO_outputs_reset(HIGH);
	IO_outputs_A1ENBL(HIGH);	// A1ENABL = 1.

	IO_read_fault1(&pin);
	return(pin);
}
//------------------------------------------------------------------------------------
uint8_t OUTB_0(void)
{
	// La salida OUT_0 corresponde al canal A (A1,A2) del DRV8814_1
	// Por norma ponemos la salida AOUT1 en 1 y AOUT2 en 0 ( APHASE = 1 ).
	// Debemos habilitar antes que nada la linea del OUT0.
	// De la Table 2. H-Bridge Logic,

uint8_t pin;

	OUTPUT_DRV_enable();
	IO_outputs_B1PHASE(HIGH); 	// Phase = 1: AOUT1->H, AOUT2->L
	IO_outputs_sleep(HIGH);
	IO_outputs_reset(HIGH);
	IO_outputs_B1ENBL(HIGH);	// A1ENABL = 1.

	IO_read_fault1(&pin);
	return(pin);

}
//------------------------------------------------------------------------------------
uint8_t OUTB_1(void)
{

uint8_t pin;

	OUTPUT_DRV_enable();
	IO_outputs_B1PHASE(LOW); 	// Phase = 1: AOUT1->H, AOUT2->L
	IO_outputs_sleep(HIGH);
	IO_outputs_reset(HIGH);
	IO_outputs_B1ENBL(HIGH);	// A1ENABL = 1.

	IO_read_fault1(&pin);
	return(pin);
}
//------------------------------------------------------------------------------------
