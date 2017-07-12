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
 *  Nueva version:
 *  Mido el tiempo entre flancos con lo que calculo en forma mas exacta el caudal, como el volumen
 *  por pulso dividido el tiempo medido.
 *  Como tengo 2 entradas digitales, utilizo 2 relojes.
 */


#include <sp5KV5.h>

static void pv_clearQ(void);
static void pv_pollQ(void);
bool pv_procesar_pulso(uint8_t channel);
static void pv_update_clock(uint8_t channel);

static char dIn_printfBuff[CHAR128];	// Buffer de impresion
static dinData_t digIn;					// Estructura local donde cuento los pulsos.

struct {
	float clock[NRO_DIGITAL_CHANNELS];			// reloj en 0.1s
	float dT[NRO_DIGITAL_CHANNELS];				// tiempo entre pulsos
	uint8_t level[NRO_DIGITAL_CHANNELS];		// nivel logico de la entrada
	uint16_t pulsos[NRO_DIGITAL_CHANNELS];		// Cantidad de pulsos contados
	bool primer_pulso[NRO_DIGITAL_CHANNELS];
} pv_Qdata;


//------------------------------------------------------------------------------------
void tkDigitalIn(void * pvParameters)
{

( void ) pvParameters;
uint8_t i;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("starting tkDigitalIn..\r\n\0"));
	FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );

	// Inicializo los latches borrandolos
	pv_clearQ();

	for ( i = 0; i < NRO_DIGITAL_CHANNELS; i++ ) {
		pv_Qdata.clock[i] = 0.0;
		pv_Qdata.dT[i] = 0;
		pv_Qdata.level[i] = 0;
		pv_Qdata.pulsos[i] = 0;
		pv_Qdata.primer_pulso[i] = true;
	}

	for( ;; )
	{

		u_kick_Wdg(WDG_DIN);

		// Espero hasta 100ms por un mensaje.
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

		// Incremento c/clock en 0.1s.
		pv_update_clock(0);
		pv_update_clock(1);

		// Solo poleo las entradas en modo normal. En modo service no para
		// poder manejarlas por los comandos de servicio.
		if ( ( systemVars.wrkMode == WK_NORMAL) || ( systemVars.wrkMode == WK_MONITOR_FRAME ))  {
			pv_pollQ();
		}
	}

}
//------------------------------------------------------------------------------------
static void pv_update_clock(uint8_t channel)
{
	// incremento el clock con el que cuento el tiempo del periodo de un pulso.
	pv_Qdata.clock[channel] += 0.1;

	// Si el tiempo es muy largo, supongo que no viene mas pulsos.
	// El criterio es que lo menos que mido es 1mt3/h. Si paso este tiempo y no llego
	// nada, es que no hay agua y por lo tanto dT = 0.

	// Por si hay problemas de configuracion.
	if ( systemVars.magPP[channel] == 0 ) {
		pv_Qdata.dT[channel] = 0.0;
		pv_Qdata.clock[channel] = 0;
		return;
	}

	if ( pv_Qdata.clock[channel] > ( systemVars.magPP[channel] * 3600 ) ) {
		pv_Qdata.dT[channel] = 0.0;
		pv_Qdata.clock[channel] = 0;

		snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("%s dDATA::clk: pulse too long (ch=%d) !!\r\n\0"), u_now(), channel );
		u_debugPrint(( D_BASIC + D_DIGITAL), dIn_printfBuff, sizeof(dIn_printfBuff) );

	}

}
//------------------------------------------------------------------------------------
static void pv_pollQ(void)
{

	// Leo los latches. Si estan en 0 es que latchearon un pulso por lo que incremento
	// los contadores.
	// Al salir los reseteo.

bool debugQ = false;
bool retS;
uint8_t pos = 0;

	// Leo el GPIO.
	retS = IO_read_pulseInputs( &pv_Qdata.level[0], &pv_Qdata.level[1] );
	if ( retS ) {

		debugQ = false;
		debugQ |= pv_procesar_pulso(0);
		debugQ |= pv_procesar_pulso(1);
	} else {
		snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("%s dDATA::poll: READ DIN ERROR !!\r\n\0"), u_now() );
		u_debugPrint(( D_BASIC + D_DIGITAL), dIn_printfBuff, sizeof(dIn_printfBuff) );
		goto quit;
	}

	if ( ((systemVars.debugLevel & D_DIGITAL) != 0) && debugQ ) {
		pos = snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("%s dDATA::poll: "),u_now());
		pos += snprintf_P( &dIn_printfBuff[pos],sizeof(dIn_printfBuff),PSTR("{p0=%d,dT0=%.1f,ck0=%.1f}"),pv_Qdata.pulsos[0], pv_Qdata.dT[0], pv_Qdata.clock[0] );
		pos += snprintf_P( &dIn_printfBuff[pos],sizeof(dIn_printfBuff),PSTR(" {p1=%d,dT1=%.1f,ck1=%.1f}\r\n\0"),pv_Qdata.pulsos[1], pv_Qdata.dT[1], pv_Qdata.clock[1]);
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
bool pv_procesar_pulso(uint8_t channel)
{
	// Si detecte un flanco puedo calcular el caudal instantaneo
	// En este caso retorno true para luego mostrar los debugs.
	// Si no detecte un flanco retorno false porque no tengo nada que mostrar.

	if ( pv_Qdata.level[channel] == 0 ) {

		pv_Qdata.dT[channel] = pv_Qdata.clock[channel];		// guardo una copia del elapsed time
		pv_Qdata.clock[channel] = 0.0;						// y lo pongo en 0 para el proximo intervalo

		if ( pv_Qdata.primer_pulso[channel] ) {			// Evito el primer pulso para no generar caudales erroneos.
			pv_Qdata.primer_pulso[channel] = false;		// Necesito 2 pulsos para calcular el tiempo entre ellos.
			return(false);
		}

		// Cuento los pulsos
		pv_Qdata.pulsos[channel]++;

		return(true);
	} else {
		return(false);
	}

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void u_readDigitalCounters( dinData_t *dIn , bool resetCounters )
{
	// copio los valores de los contadores en la estructura dIn.
	// Si se solicita, luego se ponen a 0.

	memcpy( dIn, &digIn, sizeof(dinData_t)) ;
	dIn->pulse_count[0] = pv_Qdata.pulsos[0];
	dIn->pulse_count[1] = pv_Qdata.pulsos[1];
	dIn->pulse_period[0] = pv_Qdata.dT[0];
	dIn->pulse_period[1] = pv_Qdata.dT[1];

	// Calculo los caudales
	dIn->caudal[0] = 0;
	if ( pv_Qdata.pulsos[0] > 20 ) {
		// Calculo el caudal por pulsos
		dIn->caudal[0] = pv_Qdata.pulsos[0] * systemVars.magPP[0] * 3600 / systemVars.timerPoll;
		dIn->metodo_medida[0] = 'p';
	} else {
		// Calculo el caudal por delta_T
		if ( (systemVars.magPP[0] != 0) && ( dIn->pulse_period[0] != 0 ) ) {
			dIn->caudal[0] = systemVars.magPP[0] * 3600 / dIn->pulse_period[0];
		}
		dIn->metodo_medida[0] = 't';
	}

	dIn->caudal[1] = 0;
	if ( pv_Qdata.pulsos[1] > 20 ) {
		// Calculo el caudal por pulsos
		dIn->caudal[1] = pv_Qdata.pulsos[1] * systemVars.magPP[1] * 3600 / systemVars.timerPoll;
		dIn->metodo_medida[1] = 'p';
	} else {
		// Calculo el caudal por delta_T
		if ( (systemVars.magPP[1] != 0) && ( dIn->pulse_period[1] != 0 ) ) {
			dIn->caudal[1] = systemVars.magPP[1] * 3600 / dIn->pulse_period[1];
		}
		dIn->metodo_medida[1] = 't';
	}

	if ( resetCounters == true ) {
		pv_Qdata.pulsos[0] = 0;
		pv_Qdata.pulsos[1] = 0;
	}
}
//------------------------------------------------------------------------------------
