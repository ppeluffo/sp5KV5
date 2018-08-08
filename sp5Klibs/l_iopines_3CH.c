/*
 * l_iopines.c
 *
 *  Created on: 18 de jun. de 2017
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#include <l_iopines_3CH.h>
#include "sp5KV5.h"

#ifdef SP5KV5_3CH
//------------------------------------------------------------------------------------
bool IO_read_pulseInputs( uint8_t *din0, uint8_t *din1 )
{

	// Las entradas de los pulsos ( latches ) corresponden a las entradas GPB5 y GPB6 del
	// MCP23018.

bool retS;
uint8_t regValue;

	retS = MCP_read( MCP1_ADDR, MCP1_GPIOB, &regValue);
	*din0 = ( regValue & 0x40) >> 6;
	*din1 = ( regValue & 0x20) >> 5;
	return(retS);
}
//------------------------------------------------------------------------------------
uint8_t IO_read_terminal_pin(void)
{
	// El TERMSW se cablea a PD7.
	return ( ( TERMSW_PIN & _BV(7) ) >> 7 );
}
//------------------------------------------------------------------------------------
void IO_init_pines(void)
{
	// Configuracion de pines:
	// Los pines del micro que resetean los latches de caudal son salidas.

	sbi(Q_DDR, Q0_CTL_PIN);
	sbi(Q_DDR, Q1_CTL_PIN);

	// El pin de control de la terminal es entrada
	cbi(TERMSW_DDR, TERMSW_BIT);

	// El pin de DCD es entrada
	//cbi(DCD_DDR, DCD_BIT);

	// Leds
	sbi(LED_KA_DDR, LED_KA_BIT);		// El pin del led de KA ( PD6 ) es una salida.
	sbi(LED_MODEM_DDR, LED_MODEM_BIT);	// El pin del led de KA ( PD6 ) es una salida.
	// inicialmente los led quedan en 0
	sbi(LED_KA_PORT, LED_KA_BIT);
	sbi(LED_MODEM_PORT, LED_MODEM_BIT);

	// PB0 indica el nivel digital de la entrada digital 1. (entrada)
	cbi(DL1_DDR, DL1_BIT );


}
//------------------------------------------------------------------------------------
bool IO_read_din0( uint8_t *pin)
{
bool retS;
uint8_t regValue;

	retS = MCP_read( MCP1_ADDR, MCP1_GPIOB, &regValue);
	*pin = ( regValue & 0x40) >> 6;
	return(retS);
}
//------------------------------------------------------------------------------------
bool IO_read_din1( uint8_t *pin)
{

bool retS;
uint8_t regValue;

	retS = MCP_read( MCP1_ADDR, MCP1_GPIOB, &regValue);
	*pin = ( regValue & 0x20) >> 5;
	return(retS);
}
//------------------------------------------------------------------------------------
bool IO_read_dcd( uint8_t *pin)
{
	// MCP23008 logic
bool retS;
uint8_t regValue;

	// DCD es el bit1, mask = 0x02
	retS = MCP_read( MCP0_ADDR, MCP0_GPIO, &regValue);
//	*pin = ( regValue & 0x02) >> 1;
	*pin = ( regValue & _BV(1) ) >> 1;		// bit1, mask = 0x02
	return(retS);
}
//------------------------------------------------------------------------------------
bool IO_read_fault1( uint8_t *pin)
{

bool retS;
uint8_t regValue;

	retS = MCP_read( MCP1_ADDR, MCP1_GPIOA, &regValue);
	*pin = ( regValue & 0x80) >> 7;
	return(retS);
}
//------------------------------------------------------------------------------------
#endif /* SP5KV5_3CH */
