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
static void pv_procesar_caudales(uint8_t channel);

static char dIn_printfBuff[CHAR128];	// Buffer de impresion

struct {
	uint32_t t_start[NRO_DIGITAL_CHANNELS];		// Incio de pulso
	uint8_t level[NRO_DIGITAL_CHANNELS];		// nivel logico de la entrada

	uint16_t pulse_count[NRO_DIGITAL_CHANNELS];
	float pulse_period[NRO_DIGITAL_CHANNELS];
	float caudal[NRO_DIGITAL_CHANNELS];
	char metodo_medida[NRO_DIGITAL_CHANNELS];

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
		pv_Qdata.t_start[i] = ticks;
		pv_Qdata.level[i] = 0;

		pv_Qdata.pulse_count[i] = 0;
		pv_Qdata.pulse_period[i] = 0.0;
		pv_Qdata.caudal[i] = 0.0;
		pv_Qdata.metodo_medida[i] = 'x';
	}

	for( ;; )
	{

		u_kick_Wdg(WDG_DIN);

		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

		// Poleo las entradas

		pv_pollQ();
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
		pos += snprintf_P( &dIn_printfBuff[pos],sizeof(dIn_printfBuff),PSTR("{p0=%d,dT0=%.1f}"),pv_Qdata.pulse_count[0], pv_Qdata.pulse_period[0] );
		pos += snprintf_P( &dIn_printfBuff[pos],sizeof(dIn_printfBuff),PSTR(" {p1=%d,dT1=%.1f}\r\n\0"),pv_Qdata.pulse_count[1], pv_Qdata.pulse_period[1]);
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
	// Si detecte un flanco ( pulso) incremento el contador de pulsos y el tiempo entre pulsos
	// En este caso retorno true para luego mostrar los debugs.
	// Si no detecte un flanco retorno false porque no tengo nada que mostrar.

	if ( pv_Qdata.level[channel] == 0 ) {		// Los flancos que detecto son de bajada.

		// incremento los pulsos
		pv_Qdata.pulse_count[channel]++;

		// Mido el tiempo desde el pulso anterior.
		pv_Qdata.pulse_period[channel] = (ticks - pv_Qdata.t_start[channel]) * 0.01 ;	// elapsed time
		pv_Qdata.t_start[channel] = ticks;

		return(true);

	} else {
		return(false);
	}

}
//------------------------------------------------------------------------------------
static void pv_procesar_caudales(uint8_t channel)
{
	// Llego una senal de tkAnalog que expiro el timerPoll por lo que se deben
	// hacer los calculos de caudal para el intervalo dado.

float Q_x_pulsos, Q_x_tiempo;

	// Caudal por pulsos.
	Q_x_pulsos = 0.0;
	if ( systemVars.timerPoll != 0 ) {
		Q_x_pulsos = pv_Qdata.pulse_count[channel] * systemVars.magPP[0] * 3600 / systemVars.timerPoll;
	}

	// Caudal por tiempo.
	Q_x_tiempo = 0.0;
	if ( ( systemVars.magPP[channel] > 0) && ( pv_Qdata.pulse_period[channel] > 0 ) ) {
		Q_x_tiempo = systemVars.magPP[channel] * 3600 / pv_Qdata.pulse_period[channel];
	}
	if ( Q_x_tiempo < 1 ) {			// Si el caudal es muy bajo < 1, queda en 0.
		Q_x_tiempo = 0.0;
	}

	// Veo cual caudal usar
	pv_Qdata.caudal[channel] = 0;
	if ( pv_Qdata.pulse_count[channel] > 20 ) {

		// pulsos
		pv_Qdata.caudal[channel] = Q_x_pulsos;
		pv_Qdata.metodo_medida[channel] = 'p';

	} else {

		// delta_T
		pv_Qdata.caudal[channel] = Q_x_tiempo;
		pv_Qdata.metodo_medida[channel] = 't';
	}

	// Reseteo los contadores.
	pv_Qdata.pulse_count[channel] = 0;
	pv_Qdata.pulse_period[channel] = 0;

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void u_readDigitalCounters( dinData_t *dIn )
{
	// Esta funcion es invocada SOLO desde tkDigital que es la que marca el ritmo
	// del poleo.
	// Calcula los caudales y copio los valores de los contadores en la estructura dIn.
	// De este modo quedan sincronizados los valores digitales al intervalo en que se midieron.

uint16_t pos;

	if ( (systemVars.debugLevel & D_DIGITAL) != 0)  {
		pos = snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff), PSTR("%s dDATA::frame " ), u_now() );
		pos += snprintf_P( &dIn_printfBuff[pos],sizeof(dIn_printfBuff),PSTR("{p0=%d,dT0=%.1f}"),pv_Qdata.pulse_count[0], pv_Qdata.pulse_period[0] );
		pos += snprintf_P( &dIn_printfBuff[pos],sizeof(dIn_printfBuff),PSTR(" {p1=%d,dT1=%.1f}\r\n\0"),pv_Qdata.pulse_count[1], pv_Qdata.pulse_period[1]);
		u_debugPrint( D_DIGITAL, dIn_printfBuff, sizeof(dIn_printfBuff) );
	}

	pv_procesar_caudales(0);
	pv_procesar_caudales(1);

	dIn->caudal[0] = pv_Qdata.caudal[0];
	dIn->caudal[1] = pv_Qdata.caudal[1];
	dIn->metodo_medida[0] = pv_Qdata.metodo_medida[0];
	dIn->metodo_medida[1] = pv_Qdata.metodo_medida[1];

}
//------------------------------------------------------------------------------------
