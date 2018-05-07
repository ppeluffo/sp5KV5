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
	uint8_t level[NRO_DIGITAL_CHANNELS];		// nivel logico de la entrada
	uint16_t pulse_count[NRO_DIGITAL_CHANNELS];
	float caudal[NRO_DIGITAL_CHANNELS];

} pv_Qdata;

// La tarea pasa por el mismo lugar c/100ms.
#define WDG_DIN_TIMEOUT	10

//------------------------------------------------------------------------------------
void tkDigitalIn(void * pvParameters)
{

( void ) pvParameters;
uint8_t i;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	FRTOS_snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("starting tkDigitalIn..\r\n\0"));
	FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );

	// Inicializo los latches borrandolos
	pv_clearQ();

	for ( i = 0; i < NRO_DIGITAL_CHANNELS; i++ ) {
		pv_Qdata.level[i] = 0;
		pv_Qdata.pulse_count[i] = 0;
		pv_Qdata.caudal[i] = 0.0;
	}

	for( ;; )
	{

		pub_control_watchdog_kick(WDG_DIN, WDG_DIN_TIMEOUT);

		// Cada 200 ms leo las entradas digitales. fmax=5Hz
		vTaskDelay( ( TickType_t)( 200 / portTICK_RATE_MS ) );

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

	// Leo el GPIO.
	retS = IO_read_pulseInputs( &pv_Qdata.level[0], &pv_Qdata.level[1] );

	if ( retS ) {
		debugQ = false;
		debugQ |= pv_procesar_pulso(0);
		debugQ |= pv_procesar_pulso(1);
	} else {
		FRTOS_snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("DIGITAL: poll READ DIN ERROR !!\r\n\0"));
		FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );
		goto quit;
	}

	if ( (systemVars.debugLevel == D_DIGITAL) && debugQ ) {
		FRTOS_snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("DIGITAL: poll {p0=%d,p1=%d}\r\n\0"),pv_Qdata.pulse_count[0], pv_Qdata.pulse_count[1] );
		FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );
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
	// Si detecte un flanco ( pulso) incremento el contador de pulsos.
	// En este caso retorno true para luego mostrar los debugs.
	// Si no detecte un flanco retorno false porque no tengo nada que mostrar.

	if ( pv_Qdata.level[channel] == 0 ) {		// Los flancos que detecto son de bajada.
		// incremento los pulsos
		pv_Qdata.pulse_count[channel]++;
		return(true);

	} else {
		return(false);
	}

}
//------------------------------------------------------------------------------------
static void pv_procesar_caudales(uint8_t channel)
{
	// Cuando tkAnalog lee los caudales invoca a pub_digital_read_counters().
	// Esta funcion convierte los pulsos a caudal usando esta funcion.

float caudal = 0.0;

	// Caudal por pulsos.
	if ( systemVars.timerPoll != 0 ) {
		caudal = pv_Qdata.pulse_count[channel] * systemVars.magPP[channel] * 3600 / systemVars.timerPoll;
	}

	pv_Qdata.caudal[channel] = caudal;

	/*
	if ( systemVars.debugLevel == D_DIGITAL)  {
		FRTOS_snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("DIGITAL: ch%d, pulsos=%d, factor=%.02f, caudal=%.02f\r\n\0"),channel, pv_Qdata.pulse_count[channel], systemVars.magPP[channel], caudal );
		FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );
	}
	*/

	// Reseteo el contador de pulsos del canal para el proximo intervalo.
	pv_Qdata.pulse_count[channel] = 0;

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void pub_digital_read_counters( dinData_t *dIn )
{
	// Esta funcion es invocada SOLO desde tkDigital que es la que marca el ritmo
	// del poleo.
	// Calcula los caudales y copio los valores de los contadores en la estructura dIn.
	// De este modo quedan sincronizados los valores digitales al intervalo en que se midieron.

	pv_procesar_caudales(0);
	pv_procesar_caudales(1);

	dIn->caudal[0] = pv_Qdata.caudal[0];
	dIn->caudal[1] = pv_Qdata.caudal[1];

}
//------------------------------------------------------------------------------------
bool pub_digital_config_channel( uint8_t channel, char *chName, char *s_magPP )
{
	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	if ( chName != NULL ) {
		memset ( systemVars.dChName[channel], '\0',   PARAMNAME_LENGTH );
		memcpy( systemVars.dChName[channel], chName , ( PARAMNAME_LENGTH - 1 ));
	}

	if ( s_magPP != NULL ) {
		if ( atof(s_magPP) == 0 ) {
			systemVars.magPP[channel] = 0.1;
		} else {
			systemVars.magPP[channel] = atof(s_magPP);
		}
	}

	xSemaphoreGive( sem_SYSVars );
	return(true);

}
//----------------------------------------------------------------------------------------
void pub_digital_load_defaults(void)
{
	// Realiza la configuracion por defecto de los canales digitales.
	strncpy_P(systemVars.dChName[0], PSTR("v0\0"),3);
	systemVars.magPP[0] = 0.1;
	strncpy_P(systemVars.dChName[1], PSTR("v1\0"),3);
	systemVars.magPP[1] = 0.1;

}
//------------------------------------------------------------------------------------
