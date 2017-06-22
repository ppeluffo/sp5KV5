/*
 * mcp_sp5KFRTOS.c
 *
 *  Created on: 01/11/2013
 *      Author: root
 *
 * Funciones para uso de los MCP del SP5K modificadas para usarse con FRTOS.
 *
 *
 */
//------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#ifndef AVRLIBFRTOS_MCP_SP5KFRTOS_H_
#define AVRLIBFRTOS_MCP_SP5KFRTOS_H_

#include "FRTOS-IO.h"
#include "sp5K_i2c.h"

//#define mcp_DEBUG

#ifdef mcp_DEBUG
	#define DEBUG_MCP 5
#else
	#define DEBUG_MCP 0
#endif
//------------------------------------------------------------------------------------
// MCP23008

// MCP0: MCP23008 placa logica
// MCP1: MCP23008 placa analogica

// Pines del micro ATmega1284 conectado a la interrupcion de los MCP.
#define MCP0_PORT	PORTD
#define MCP0_PIN	PIND
#define MCP0_BIT	5
#define MCP0_DDR	DDRD
#define MCP0_MASK	0x20

#define MCP1_PORT	PORTB
#define MCP1_PIN	PINB
#define MCP1_BIT	2
#define MCP1_DDR	DDRB
#define MCP1_MASK	0x04

//------------------------------------------------------------------------------------
// Identificacion en el bus I2C de los MCP
#define MCP0_ADDR			0x40	// MCP23008
#define MCP1_ADDR			0x4E	// MCP23018
//------------------------------------------------------------------------------------
// Registros MCP0
#define MCP0_IODIR		0x00
#define MCP0_IPOL		0x01
#define MCP0_GPINTEN	0x02
#define MCP0_DEFVAL		0x03
#define MCP0_INTCON		0x04
#define MCP0_IOCON		0x05
#define MCP0_GPPU		0x06
#define MCP0_INTF		0x07
#define MCP0_INTCAP		0x08
#define MCP0_GPIO 		0x09
#define MCP0_OLAT 		0x0A

// Bits del MCP0
#define MCP0_GPIO_IGPRSDCD			1	// IN
#define MCP0_GPIO_IGPRSRI			2	// IN
#define MCP0_GPIO_OGPRSSW			3	// OUT
#define MCP0_GPIO_OTERMPWR			4
#define MCP0_GPIO_OGPRSPWR			5
#define MCP0_GPIO_OLED				6

//------------------------------------------------------------------------------------
// Registros MCP1
#define MCP1_IODIRA					0x00
#define MCP1_IODIRB					0x01
#define MCP1_GPINTENA				0x04
#define MCP1_GPINTENB				0x05
#define MCP1_DEFVALA				0x06
#define MCP1_DEFVALB				0x07
#define MCP1_INTCONA				0x08
#define MCP1_INTCONB				0x09
#define MCP1_IOCON					0x0A
#define MCP1_GPPUA					0x0C
#define MCP1_GPPUB					0x0D
#define MCP1_INTFA					0x0E
#define MCP1_INTFB					0x0F
#define MCP1_INTCAPA				0x10
#define MCP1_INTCAPB				0x11
#define MCP1_GPIOA					0x12
#define MCP1_GPIOB					0x13
#define MCP1_OLATA					0x14
#define MCP1_OLATB					0x15

// Bits del MCP1
#define MCP1_ENA2						0
#define MCP1_ENB2						1
#define MCP1_PHB2						2
#define MCP1_PHB1						3
#define MCP1_ENB1						4
#define MCP1_ENA1						5
#define MCP1_PHA1						6
#define MCP1_RESET						0
#define MCP1_SLEEP						1
#define MCP1_FAULT						2
#define MCP1_PHA2						3
#define MCP1_PWRSENSORS					4
#define MCP1_DIN0						5
#define MCP1_DIN1						6
#define MCP1_OANALOG					7

#define MCP1_GPIO_DIN0					6	// IN
#define MCP1_GPIO_DIN1					5	// IN
#define MCP1_GPIO_PWRSENSORS			4	// OUT
#define MCP1_GPIO_ANALOGPWR				7	// OUT
//------------------------------------------------------------------------------------

void MCP_init(uint8_t device_id );
bool MCP_write( uint8_t deviceId, uint8_t byteAddr, uint8_t value );
bool MCP_read( uint8_t deviceId, uint8_t byteAddr, uint8_t *retValue );
bool MCP_modify( uint8_t deviceId, uint8_t byteAddr, uint8_t value, uint8_t bitMask );
//
#endif /* AVRLIBFRTOS_MCP_SP5KFRTOS_H_ */
