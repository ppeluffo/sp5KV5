/*
 * l_iopines.c
 *
 *  Created on: 18 de jun. de 2017
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#include <l_iopines.h>
#include "sp5KV5.h"

// --------------------------------------------------------------------------------
void IO_outputs_reset(t_low_high level)
{

	if ( level == HIGH) {
		MCP_modify( MCP1_ADDR, MCP1_OLATB, 1, MCP1_RESET );
	} else {
		MCP_modify( MCP1_ADDR, MCP1_OLATB, 0, MCP1_RESET );
	}
}
// --------------------------------------------------------------------------------
void IO_outputs_sleep(t_low_high level)
{

	if ( level == HIGH) {
		MCP_modify( MCP1_ADDR, MCP1_OLATB, 1, MCP1_SLEEP );
	} else {
		MCP_modify( MCP1_ADDR, MCP1_OLATB, 0, MCP1_SLEEP );
	}
}
//------------------------------------------------------------------------------------
void IO_outputs_A1ENBL(t_low_high level)
{

	if ( level == HIGH) {
		MCP_modify( MCP1_ADDR, MCP1_OLATA, 1, MCP1_ENA1 );
	} else {
		MCP_modify( MCP1_ADDR, MCP1_OLATA, 0, MCP1_ENA1 );
	}
}
//------------------------------------------------------------------------------------
void IO_outputs_B1ENBL(t_low_high level)
{

	if ( level == HIGH) {
		MCP_modify( MCP1_ADDR, MCP1_OLATA, 1, MCP1_ENB1 );
	} else {
		MCP_modify( MCP1_ADDR, MCP1_OLATA, 0, MCP1_ENB1 );
	}
}
//------------------------------------------------------------------------------------
void IO_outputs_A1PHASE(t_low_high level){

	if ( level == HIGH) {
		MCP_modify( MCP1_ADDR, MCP1_OLATA, 1, MCP1_PHA1 );
	} else {
		MCP_modify( MCP1_ADDR, MCP1_OLATA, 0, MCP1_PHA1 );
	}
}
//------------------------------------------------------------------------------------
void IO_outputs_B1PHASE(t_low_high level){

	if ( level == HIGH) {
		MCP_modify( MCP1_ADDR, MCP1_OLATA, 1, MCP1_PHB1 );
	} else {
		MCP_modify( MCP1_ADDR, MCP1_OLATA, 0, MCP1_PHB1 );
	}
}
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
uint8_t IO_read_tilt_pin(void)
{
	return( ( TILT_PIN & _BV(0) ) >> 0 );

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

	// El pin de TILT es entrada
	cbi(TILT_DDR, TILT_BIT);

	// Leds
	sbi(LED_KA_DDR, LED_KA_BIT);		// El pin del led de KA ( PD6 ) es una salida.
	sbi(LED_MODEM_DDR, LED_MODEM_BIT);	// El pin del led de KA ( PD6 ) es una salida.
	// inicialmente los led quedan en 0
	sbi(LED_KA_PORT, LED_KA_BIT);
	sbi(LED_MODEM_PORT, LED_MODEM_BIT);
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
