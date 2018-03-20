/*
 * l_iopines.h
 *
 *  Created on: 18 de jun. de 2017
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#ifndef SRC_SP5KLIBS_L_IOPINES_H_
#define SRC_SP5KLIBS_L_IOPINES_H_

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

// Led de placa analogica ( PD6 )
#define LED_KA_PORT		PORTD
#define LED_KA_PIN		PIND
#define LED_KA_BIT		6
#define LED_KA_DDR		DDRD
#define LED_KA_MASK		0x40

// Led del modem en placa analogica
#define LED_MODEM_PORT		PORTC
#define LED_MODEM_PIN		PINC
#define LED_MODEM_BIT		3
#define LED_MODEM_DDR		DDRC
#define LED_MODEM_MASK		0x04

// TERMINAL
// Pin de control de fuente de la terminal ( PD7)
#define TERMSW_PORT		PORTD
#define TERMSW_PIN		PIND
#define TERMSW_BIT		7
#define TERMSW_DDR		DDRD
#define TERMSW_MASK		0x80

// Sensor de TILT
#define TILT_PORT		PORTB
#define TILT_PIN		PINB
#define TILT_BIT		0
#define TILT_DDR		DDRB
#define TILT_MASK		0x1

// Q PINES
#define Q_PORT		PORTA
#define Q_DDR		DDRA
#define Q0_CTL_PIN	2
#define Q1_CTL_PIN	3

// DCD
// Como el MCP23018 a veces no detecta el nivel del modem, cableamos
// el DCD a PB3
// Pin de control de fuente de la terminal ( PB3)
#define DCD_PORT		PORTB
#define DCD_PIN			PINB
#define DCD_BIT			3
#define DCD_DDR			DDRB
#define DCD_MASK		0x8

// --------------------------------------------------------------------------------

#define IO_set_led_KA_logicBoard() 		(MCP_modify( MCP0_ADDR, MCP0_GPIO, 1, MCP0_GPIO_OLED ))
#define IO_clear_led_KA_logicBoard() 	(MCP_modify( MCP0_ADDR, MCP0_GPIO, 0, MCP0_GPIO_OLED ))
#define IO_set_led_KA_analogBoard() 	(cbi(LED_KA_PORT, LED_KA_BIT))
#define IO_clear_led_KA_analogBoard() 	(sbi(LED_KA_PORT, LED_KA_BIT))

#define IO_set_led_MODEM_analogBoard() 	(cbi(LED_MODEM_PORT, LED_MODEM_BIT))
#define IO_clear_led_MODEM_analogBoard() (sbi(LED_MODEM_PORT, LED_MODEM_BIT))

#define IO_term_pwr_on() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 1, MCP0_GPIO_OTERMPWR ) )
#define IO_term_pwr_off() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 0, MCP0_GPIO_OTERMPWR ) )

#define IO_sensor_pwr_on() 	( MCP_modify( MCP1_ADDR, MCP1_OLATB, 1, MCP1_PWRSENSORS ) )
#define IO_sensor_pwr_off() ( MCP_modify( MCP1_ADDR, MCP1_OLATB, 0, MCP1_PWRSENSORS ) )

#define IO_analog_pwr_on() 	( MCP_modify( MCP1_ADDR, MCP1_OLATB, 1, MCP1_OANALOG ) )
#define IO_analog_pwr_off() ( MCP_modify( MCP1_ADDR, MCP1_OLATB, 0, MCP1_OANALOG ) )

#define IO_modem_hw_pwr_on() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 1, MCP0_GPIO_OGPRSPWR ) )
#define IO_modem_hw_pwr_off() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 0, MCP0_GPIO_OGPRSPWR ) )

// El modem_sw_switch es un transistor por lo tanto invierte la entrada en la salida. !!!
#define IO_modem_sw_switch_high() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 0, MCP0_GPIO_OGPRSSW ) )
#define IO_modem_sw_switch_low() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 1, MCP0_GPIO_OGPRSSW ) )

#define IO_set_Q0()		( sbi(Q_PORT, Q0_CTL_PIN))
#define IO_clear_Q0()	( cbi(Q_PORT, Q0_CTL_PIN))

#define IO_set_Q1()		( sbi(Q_PORT, Q1_CTL_PIN))
#define IO_clear_Q1()	( cbi(Q_PORT, Q1_CTL_PIN))

void IO_outputs_reset(t_low_high level);
void IO_outputs_sleep(t_low_high level);
void IO_outputs_A1ENBL(t_low_high level);
void IO_outputs_B1ENBL(t_low_high level);
void IO_outputs_A1PHASE(t_low_high level);
void IO_outputs_B1PHASE(t_low_high level);

bool IO_read_pulseInputs( uint8_t *din0, uint8_t *din1 );
uint8_t IO_read_terminal_pin(void);
uint8_t IO_read_tilt_pin(void);
void IO_init_pines(void);
bool IO_read_din0( uint8_t *pin);
bool IO_read_din1( uint8_t *pin);
bool IO_read_dcd( uint8_t *pin);
bool IO_read_fault1( uint8_t *pin);

#endif /* SRC_SP5KLIBS_L_IOPINES_H_ */
