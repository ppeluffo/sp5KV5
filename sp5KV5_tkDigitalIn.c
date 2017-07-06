/*
 * sp5KV3_tkDigitalIn.c
 *
 *  Created on: 13/4/2015
 *      Author: pablo
 *
 *  La nueva modalidad es por poleo.
 *  Configuro el MCP para que no interrumpa
 *  C/100ms leo el registro GPIO del MCP.
 *  En reposo la salida de los latch es 1 por lo que debo detectar cuando se hizo 0.
 *  Para evitar poder quedar colgado, c/ciclo borro el latch.
 *  Esto implica que no importa la duracion del pulso ya que lo capturo con un flip-flop, pero
 *  no pueden venir mas rapido que 10/s.
 *
 *	Esta version solo puede usarse con placas SP5K_3CH que tengan latch para los pulsos, o sea
 *	version >= R003.
 *
 */


#include <sp5KV5.h>

static void pv_clearQ(void);
static void pv_pollQ(void);

static char dIn_printfBuff[CHAR64];	// Buffer de impresion
static dinData_t digIn;				// Estructura local donde cuento los pulsos.

//------------------------------------------------------------------------------------
void tkDigitalIn(void * pvParameters)
{

( void ) pvParameters;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("starting tkDigitalIn..\r\n\0"));
	FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );

	// Inicializo los latches borrandolos
	pv_clearQ();
	digIn.level[0] = 0;
	digIn.level[1] = 0;
	digIn.pulses[0] = 0;
	digIn.pulses[1] = 0;

	for( ;; )
	{

		u_kick_Wdg(WDG_DIN);

		// Espero hasta 250ms por un mensaje.
		vTaskDelay( ( TickType_t)( 250 / portTICK_RATE_MS ) );

		// Solo poleo las entradas en modo normal. En modo service no para
		// poder manejarlas por los comandos de servicio.
		if ( ( systemVars.wrkMode == WK_NORMAL) || ( systemVars.wrkMode == WK_MONITOR_FRAME ))  {
			pv_pollQ();
		}
	}

}
//------------------------------------------------------------------------------------
static void pv_pollQ(void)
{

	// Leo los latches. Si estan en 0 es que latchearon un pulso por lo que incremento
	// los contadores.
	// Al salir los reseteo.

uint8_t din0 = 0;
uint8_t din1 = 0;
bool debugQ = false;
bool retS;

	// Leo el GPIO.
	retS = IO_read_pulseInputs( &din0, &din1 );
	if ( retS ) {
		// Levels
		digIn.level[0] = din0;
		digIn.level[1] = din1;

		// Counts
		debugQ = false;
		if (din0 == 0 ) { digIn.pulses[0]++ ; debugQ = true;}
		if (din1 == 0 ) { digIn.pulses[1]++ ; debugQ = true;}
	} else {
		snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("%s dDATA::poll: READ DIN ERROR !!\r\n\0"), u_now() );
		u_debugPrint(( D_BASIC + D_DIGITAL), dIn_printfBuff, sizeof(dIn_printfBuff) );
		goto quit;
	}

	if ( ((systemVars.debugLevel & D_DIGITAL) != 0) && debugQ ) {
		snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("%s dDATA::poll: din0=%.0f(%d)[.0f],din1=%.0f(%d)[.0f]\r\n\0"),u_now(),digIn.pulses[0],din0,digIn.pulses[1],din1);
		u_debugPrint( D_DIGITAL, dIn_printfBuff, sizeof(dIn_printfBuff) );
	}


quit:
	// Siempre borro los latches para evitar la posibilidad de quedar colgado.
	pv_clearQ();
	return;

}
//------------------------------------------------------------------------------------
static void pv_clearQ(void)
{
	// Pongo un pulso 1->0->1 en Q0/Q1 pin para resetear el latch
	// En reposo debe quedar en H.

	IO_clear_Q0();
	IO_clear_Q1();

	taskYIELD();
	//_delay_us(5);
	//asm("nop");

	IO_set_Q0();
	IO_set_Q1();
}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void u_readDigitalCounters( dinData_t *dIn , bool resetCounters )
{
	// copio los valores de los contadores en la estructura dIn.
	// Si se solicita, luego se ponen a 0.

	memcpy( dIn, &digIn, sizeof(dinData_t)) ;
	if ( resetCounters == true ) {
		digIn.level[0] = 0;
		digIn.level[1] = 0;
		digIn.pulses[0] = 0;
		digIn.pulses[1] = 0;
	}
}
//------------------------------------------------------------------------------------
