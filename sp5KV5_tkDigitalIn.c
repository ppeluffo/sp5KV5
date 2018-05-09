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
static void pv_digital_init(void);

static char dIn_printfBuff[CHAR128];	// Buffer de impresion

#ifdef SP5KV5_3CH

static void pv_procesar_caudales(uint8_t channel);
bool pv_procesar_pulso(uint8_t channel);

#endif /* SP5KV5_3CH */

#ifdef SP5KV5_8CH

dinData_t pv_Qdata;
static uint16_t total_ticks;

#endif /* SP5KV5_8CH */

// La tarea pasa por el mismo lugar c/100ms.
#define WDG_DIN_TIMEOUT	10

//------------------------------------------------------------------------------------
void tkDigitalIn(void * pvParameters)
{

( void ) pvParameters;
uint32_t waiting_ticks;
TickType_t xLastWakeTime;


	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	FRTOS_snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("starting tkDigitalIn..\r\n\0"));
	FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );

	// Inicializo los latches borrandolos
	pv_digital_init();

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    // Al arrancar poleo a los 5s
    waiting_ticks = (uint32_t)(100) / portTICK_RATE_MS;

	for( ;; )
	{

		pub_control_watchdog_kick(WDG_DIN, WDG_DIN_TIMEOUT);

		// Cada 100 ms leo las entradas digitales. fmax=10Hz
		//vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		vTaskDelayUntil( &xLastWakeTime, waiting_ticks );

		// Poleo las entradas
		pv_pollQ();
	}

}
//------------------------------------------------------------------------------------
static void pv_digital_init(void)
{
uint8_t i;

	pv_clearQ();

#ifdef SP5KV5_3CH

	for ( i = 0; i < NRO_DIGITAL_CHANNELS; i++ ) {
		pv_Qdata.level[i] = 0;
		pv_Qdata.pulse_count[i] = 0;
		pv_Qdata.caudal[i] = 0.0;
	}
#endif /* SP5KV5_3CH */

#ifdef SP5KV5_8CH

	total_ticks = 0;

	for ( i = 0; i < NRO_DIGITAL_CHANNELS; i++ ) {
		pv_Qdata.level[i] = 0;
		pv_Qdata.ticks_time_H[i] = 0;
	}
#endif /* SP5KV5_8CH */

}
//------------------------------------------------------------------------------------
static void pv_pollQ(void)
{

	// Leo los latches. Si estan en 0 es que latchearon un pulso por lo que incremento
	// los contadores.
	// Al salir los reseteo.
#ifdef SP5KV5_3CH

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

#endif /* SP5KV5_3CH */

#ifdef SP5KV5_8CH

uint8_t channel;

	// Incremento los ticks dentro del intervalo.
	total_ticks++;

	// Paso 1: Actualizo los niveles logicos de las 4 entradas
	pv_Qdata.level[0] = IO_read_din0_level();
	pv_Qdata.level[1] = IO_read_din1_level();
	pv_Qdata.level[2] = IO_read_din2_level();
	pv_Qdata.level[3] = IO_read_din3_level();

	// Paso 2: Tiempo que el pin esta en LOW
	// Mido el tiempo en intervalos de 100ms que la señal esta en LOW.
	// Normalmente ( flotando ) estaria en HIGH
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++ ) {
		if ( pv_Qdata.level[channel] == 1 ) {
			pv_Qdata.ticks_time_H[channel]++;
		}
	}

#endif /* SP5KV5_8CH */

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
#ifdef SP5KV5_3CH
	IO_clear_Q0();
	IO_clear_Q1();

	taskYIELD();
	//_delay_us(5);
	//asm("nop");

	IO_set_Q0();
	IO_set_Q1();
#endif /* SP5KV5_3CH */

#ifdef SP5KV5_8CH

	IO_clr_CLRD();
	taskYIELD();
	IO_set_CLRD();

#endif /* SP5KV5_8CH */
}
//------------------------------------------------------------------------------------
#ifdef SP5KV5_3CH

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
#endif /* SP5KV5_3CH */
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void pub_digital_read_counters( dinData_t *dIn )
{
	// Esta funcion es invocada SOLO desde tkDigital que es la que marca el ritmo
	// del poleo.
	// Calcula los caudales y copio los valores de los contadores en la estructura dIn.
	// De este modo quedan sincronizados los valores digitales al intervalo en que se midieron.

#ifdef SP5KV5_3CH
	pv_procesar_caudales(0);
	pv_procesar_caudales(1);

	dIn->caudal[0] = pv_Qdata.caudal[0];
	dIn->caudal[1] = pv_Qdata.caudal[1];
#endif /* SP5KV5_3CH */

#ifdef SP5KV5_8CH
	// Copia el frame digital al puntero dst.
	// Primero debo ajustar los ticks.
	// El tema es que por ej. conte 599 ticks pero deberian habers sido teoricamente 600,
	// entonces falta 1 tick lo que indica que en algun momento la entrada estubo baja pero
	// en realidad fue que por un tema del clock, el intervalo fue unos ms. menos.
	// Para que esto no pase, debo corregirlo.
	// Teoricamente deberia haber contado systemVars.timerPoll * 10 ticks en un intervalo.
	// pero lo que realmente conte fue total_ticks.
	// Ajusto los ticks_time_H de c/entrada con estos valores.


uint8_t channel;
float ajuste_ticks = systemVars.timerPoll * 10 / total_ticks;

	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++ ) {
		dIn->level[channel] = pv_Qdata.level[channel];
		dIn->ticks_time_H[channel] = (uint16_t) ( ajuste_ticks * pv_Qdata.ticks_time_H[channel] );
		pv_Qdata.ticks_time_H[channel] = 0;
	}

//	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
//		FRTOS_snprintf_P( dIn_printfBuff, sizeof(dIn_printfBuff) ,PSTR("L%d=%d, T%d=%d\r\n\0"),channel,dIn->level[channel],channel,dIn->ticks_time_H[channel] );
//		FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );
//	}

	total_ticks = 0;

#endif /* SP5KV5_8CH */
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

#ifdef SP5KV5_3CH
	if ( s_magPP != NULL ) {
		if ( atof(s_magPP) == 0 ) {
			systemVars.magPP[channel] = 0.1;
		} else {
			systemVars.magPP[channel] = atof(s_magPP);
		}
	}
#endif /* SP5KV5_3CH */

	xSemaphoreGive( sem_SYSVars );
	return(true);

}
//----------------------------------------------------------------------------------------
void pub_digital_load_defaults(void)
{
	// Realiza la configuracion por defecto de los canales digitales.
#ifdef SP5KV5_3CH
	strncpy_P(systemVars.dChName[0], PSTR("v0\0"),3);
	systemVars.magPP[0] = 0.1;
	strncpy_P(systemVars.dChName[1], PSTR("v1\0"),3);
	systemVars.magPP[1] = 0.1;
#endif /* SP5KV5_3CH */

#ifdef SP5KV5_8CH

uint8_t channel;

	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		FRTOS_snprintf_P( systemVars.dChName[channel], PARAMNAME_LENGTH, PSTR("D%d\0"),channel );
	}
#endif /* SP5KV5_8CH */


}
//------------------------------------------------------------------------------------
void pub_digital_read_Inputs( uint8_t channel)
{
	// Leo el valor de una entrada analogica.
	// Debo determinar segun cual canal, que INA corresponde.

uint8_t pin;

#ifdef SP5KV5_3CH

	switch( channel ) {
	case 0:
		IO_read_din0(&pin);
		break;
	case 1:
		IO_read_din1(&pin);
		break;
	}
#endif /* SP5KV5_3CH */

#ifdef SP5KV5_8CH
	switch ( channel ) {
	case 0:
		pin = IO_read_din0_level();
		break;
	case 1:
		pin = IO_read_din1_level();
		break;
	case 2:
		pin = IO_read_din2_level();
		break;
	case 3:
		pin = IO_read_din3_level();
		break;
	}
#endif /* SP5KV5_8CH */

	FRTOS_snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("DIN_%d=%d\r\n\0"), channel, pin);
	FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );

}

//------------------------------------------------------------------------------------
