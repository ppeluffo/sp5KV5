/*
 * l_iopines.h
 *
 *  Created on: 18 de jun. de 2017
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#ifndef SRC_SP5KLIBS_L_IOPINES_8CH_H_
#define SRC_SP5KLIBS_L_IOPINES_8CH_H_

#include "sp5KV5_defs.h"

#ifdef SP5KV5_8CH

#include <l_mcp.h>
#include <stdbool.h>
#include "sp5Klibs/avrlibdefs.h"

typedef enum { LOW = 0, HIGH } t_low_high;

//#define io_DEBUG

#ifdef io_DEBUG
	#define DEBUG_IO 4
#else
	#define DEBUG_IO 0
#endif

// CLRD
// Pin de borrado del latch de entradas digitales. ( PD7)
#define CLRD_PORT		PORTD
#define CLRD_PIN		PIND
#define CLRD_BITPOS		7
#define CLRD_DDR		DDRD
#define CLRD_MASK		0x80

#define IO_config_CLRD()	sbi(CLRD_DDR, CLRD_BITPOS)
#define IO_set_CLRD()		(cbi(CLRD_PORT, CLRD_BITPOS))
#define IO_clr_CLRD()		(sbi(CLRD_PORT, CLRD_BITPOS))

// PINES:
#define DIN0_PORT		PORTB
#define DIN0_PIN		PINB
#define DIN0_BITPOS		4
#define DIN0_DDR		DDRB
#define DIN0_MASK		0x10

#define DIN1_PORT		PORTB
#define DIN1_PIN		PINB
#define DIN1_BITPOS		2
#define DIN1_DDR		DDRB
#define DIN1_MASK		0x08

#define DIN2_PORT		PORTB
#define DIN2_PIN		PINB
#define DIN2_BITPOS		0
#define DIN2_DDR		DDRB
#define DIN2_MASK		0x01

#define DIN3_PORT		PORTA
#define DIN3_PIN		PINA
#define DIN3_BITPOS		1
#define DIN3_DDR		DDRA
#define DIN3_MASK		0x02

// DCD
// Como el MCP23018 a veces no detecta el nivel del modem, cableamos
// el DCD a PB3
// Pin de control DCD. ( PB3)
#define DCD_PORT		PORTB
#define DCD_PIN			PINB
#define DCD_BIT			3
#define DCD_DDR			DDRB
#define DCD_MASK		0x8

// --------------------------------------------------------------------------------

#define IO_set_led_KA_logicBoard() 		(MCP_modify( MCP0_ADDR, MCP0_GPIO, 1, MCP0_GPIO_OLED ))
#define IO_clear_led_KA_logicBoard() 	(MCP_modify( MCP0_ADDR, MCP0_GPIO, 0, MCP0_GPIO_OLED ))

#define IO_term_pwr_on() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 1, MCP0_GPIO_OTERMPWR ) )
#define IO_term_pwr_off() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 0, MCP0_GPIO_OTERMPWR ) )

#define IO_modem_hw_pwr_on() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 1, MCP0_GPIO_OGPRSPWR ) )
#define IO_modem_hw_pwr_off() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 0, MCP0_GPIO_OGPRSPWR ) )

// El modem_sw_switch es un transistor por lo tanto invierte la entrada en la salida. !!!
#define IO_modem_sw_switch_high() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 0, MCP0_GPIO_OGPRSSW ) )
#define IO_modem_sw_switch_low() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 1, MCP0_GPIO_OGPRSSW ) )

bool IO_read_dcd( uint8_t *pin);

void IO_init_pines(void);
uint8_t IO_read_din0_level(void);
uint8_t IO_read_din1_level(void);
uint8_t IO_read_din2_level(void);
uint8_t IO_read_din3_level(void);

#endif /* SP5KV5_8CH */

#endif /* SRC_SP5KLIBS_L_IOPINES_8CH_H_ */
