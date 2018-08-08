/*
 * l_iopines.c
 *
 *  Created on: 18 de jun. de 2017
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#include <l_iopines_8CH.h>

#ifdef SP5KV5_8CH

// --------------------------------------------------------------------------------
void IO_init_pines(void)
{
	// Configuracion de pines:
	// Los pines del micro que resetean los latches de caudal son salidas.
	IO_config_CLRD();
	//sbi(Q_DDR, Q1_CTL_PIN);

	// El pin de control de la terminal es entrada
	//cbi(TERMSW_DDR, TERMSW_BIT);

	// El pin de DCD es entrada
	//cbi(DCD_DDR, DCD_BIT);

}
//------------------------------------------------------------------------------------
uint8_t IO_read_din0_level(void)
{
	return ( ( DIN0_PIN & _BV(DIN0_BITPOS) ) >> DIN0_BITPOS );
}
//------------------------------------------------------------------------------------
uint8_t IO_read_din1_level(void)
{
	return ( ( DIN1_PIN & _BV(DIN1_BITPOS) ) >> DIN1_BITPOS );
}
//------------------------------------------------------------------------------------
uint8_t IO_read_din2_level(void)
{
	return ( ( DIN2_PIN & _BV(DIN2_BITPOS) ) >> DIN2_BITPOS );
}
//------------------------------------------------------------------------------------
uint8_t IO_read_din3_level(void)
{
	return ( ( DIN3_PIN & _BV(DIN3_BITPOS) ) >> DIN3_BITPOS );
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
#endif /* SP5KV5_8CH */
