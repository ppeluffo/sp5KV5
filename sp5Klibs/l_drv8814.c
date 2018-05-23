/*
 * l_drv8814.c
 *
 *  Created on: 21 de oct. de 2017
 *      Author: pablo
 */

#include "l_drv8814.h"

#ifdef SP5KV5_3CH
//------------------------------------------------------------------------------------
bool DRV8814_test_pulse(char *s0, char *s1, char *s2)
{

char channel;
char phase;
uint16_t pulse_width;

	// Prendo la fuente de 12V

	// Determino el canal: A o B
	switch ( toupper(s0[0]) ) {
	case 'A': channel = 'A';
			break;
	case 'B': channel = 'B';
			break;
	default:
			return(false);
	}

	// Determino la fase: + o -
	switch (s1[0]) {
	case '+': phase = '+';
			break;
	case '-': phase = '-';
			break;
	default:
			return(false);
	}

	pulse_width = atoi(s2);

	DRV8814_pulse(channel, phase, pulse_width);

	vTaskDelay( ( TickType_t)(500 / portTICK_RATE_MS ) );

	// Apago la fuente de 12V

	return(true);
}
//------------------------------------------------------------------------------------
void DRV8814_pulse(char channel, char phase, uint16_t pulse_width)
{
	// channel_id: puede ser 0 ( valvula conectada a la salida 0, A1,A2) o 1 ( B1, B2 )
	// fase: '+' (P0=1,P1=0), '-' (P0=0,P1=1)
	// duracion: ms.de duracion del pulso.

	IO_set_SLP();
	IO_set_RES();
	vTaskDelay( ( TickType_t)(1) );

	switch(channel) {
	case 'A':	// Salidas AOUT1,AOUT2
		if ( phase == '+') {
			IO_set_PHA();
		} else if ( phase == '-') {
			IO_clr_PHA();
		} else {
			break;
		}
		IO_set_ENA();		// A1ENABL = 1.
		vTaskDelay( ( TickType_t)(pulse_width / portTICK_RATE_MS ) );
		IO_clr_ENA();		// A1ENABL = 0. disable bridge
		break;

	case 'B': // Salidas BOUT1, BOUT2
		if ( phase == '+') {
			IO_set_PHB();
		} else if ( phase == '-') {
			IO_clr_PHB();
		} else {
			break;
		}
		IO_set_ENB();		// B1ENABL = 1.
		vTaskDelay( ( TickType_t)(pulse_width / portTICK_RATE_MS ) );
		IO_clr_ENB();		// B1ENABL = 0. disable bridge
		break;
	}

	// Dejo el sistema en reposo
	IO_clr_PHA();
	IO_clr_PHB();
	IO_clr_ENA();
	IO_clr_ENB();
	IO_clr_SLP();
	IO_clr_RES();
}
//------------------------------------------------------------------------------------
void DRV8814_set( char channel, char *sequence )
{
	// Setea las terminales de las conexiones a las valvulas en un valor fijo.
	// Saca al driver del reposo y no lo vuelve a poner en el.
	// channel_id: puede ser 0 ( valvula conectada a la salida 0, A1,A2) o 1 ( B1, B2 )
	// La secuencia pueder ser "10" o "01".

	IO_set_SLP();
	IO_set_RES();
	vTaskDelay( ( TickType_t)(10) );

	switch(channel) {
	case 'A':	// Salidas AOUT1,AOUT2

		if ( !strcmp_P( strupr(sequence), PSTR("10")) ) {
			// 10 -> AOUT1=1(12V), AOUT2=0(0v)
			IO_set_PHA();
		} else {
			// 01 -> AOUT1=0(0V), AOUT2=1(12v)
			IO_clr_PHA();
		}
		IO_set_ENA();		// A1ENABL = 1.
		vTaskDelay( ( TickType_t)(10) );
		break;

	case 'B': // Salidas BOUT1, BOUT2
		if (!strcmp_P( strupr(sequence), PSTR("10")) ) {
			// 10 -> BOUT1=1(12V), BOUT2=0(0v)
			IO_clr_PHB();
		} else {
			// 01 -> BOUT1=0(0V), BOUT2=1(12v)
			IO_set_PHB();
		}
		IO_set_ENB();		// B1ENABL = 1.
		vTaskDelay( ( TickType_t)(10) );
		break;
	}
}

#endif
