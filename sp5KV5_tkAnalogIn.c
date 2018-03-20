/*
 * sp5KV3_tkAnalogIn.c
 *
 *  Created on: 14/4/2015
 *      Author: pablo
 */


#include <sp5KV5.h>

#define CICLOS_POLEO	3		// ciclos de poleo para promediar.
#define MAX_ANALOG_WAIT_TIME	0xFFFF

static char aIn_printfBuff[CHAR256];
static double rAIn[NRO_ANALOG_CHANNELS + 1];	// Almaceno los datos de conversor A/D
static frameData_t ANframe;
static uint16_t  AN_timer;			// Temporizado que indica cuando poleo
TimerHandle_t pollingTimer;

typedef enum { anPOLEAR, anPROCESAR, anESPERAR } t_anStates;

static void pv_tka_poleo(void);

static bool primer_frame;

// FUNCIONES PRIVADAS
static void pv_tka_promediar_datos(void);
static void pv_tka_save_frame_inBD(void);
static void pv_tka_signal_tasks(void);
static void pv_tka_print_frame(void);
static void pv_tka_timer_callback( TimerHandle_t pxTimer );
static void pv_init_AnFrame(void);

static uint16_t pv_tka_set_waiting_time(void);
static void pv_tka_prender_sensores(void);
static void pv_tka_apagar_sensores(void);

//-------------------------------------------------------------------------------------
void tkAnalogIn(void * pvParameters)
{

( void ) pvParameters;
BaseType_t xResult;
uint32_t ulNotifiedValue;
uint8_t an_state;

	while ( !startTask ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("starting tkAnalogIn..\r\n\0"));
	FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	//
	AN_timer = 5;				// Inicialmente espero 5s para polear
	pv_tka_apagar_sensores(); 	// Los sensores arrancan apagados
	an_state = anESPERAR;			// El primer estado al que voy a ir.
	primer_frame = true;

	if ( xTimerStart( pollingTimer, 0 ) != pdPASS ) {	// Arranco el timer
		u_panic(P_AIN_TIMERSTART);
	}

	pv_init_AnFrame();

	for( ;; )
	{

		u_kick_Wdg(WDG_AIN);

		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 250 / portTICK_RATE_MS ) );

		// Veo si llego un mensaje
		if ( xResult == pdTRUE ) {

			if ( ( ulNotifiedValue & TK_PARAM_RELOAD ) != 0 ) {			// Mensaje de reload configuration.
				pv_tka_set_waiting_time();								// Reflejo los cambios de configuracion
			} else if ( ( ulNotifiedValue & TK_READ_FRAME ) != 0 ) {  	// Mensaje de read frame desde el cmdLine.
				AN_timer = 2;
			}
		}

		// Recorro la maquina de estados
		switch(an_state) {
		case anPOLEAR:
			pv_tka_poleo();			// Leo los datos
			an_state = anPROCESAR;	// next state
			break;
		case anPROCESAR:
			pv_tka_promediar_datos();		// Promedio y corrijo el offset
			if ( primer_frame ) {
				// Descarto el primer frame.
				primer_frame = false;
				snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("%s aDATA::Primer frame clear:\r\n\0"),u_now() );
				FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
			} else {
				pv_tka_save_frame_inBD();		// Guardo en memoria
				pv_tka_signal_tasks();			// Aviso a otras tareas que hay datos
				pv_tka_print_frame();			// Log
			}
			an_state = anESPERAR;			// next state
			break;
		case anESPERAR:
			// Expiro el timer: salida normal
			if ( AN_timer == 0 ) {
				// Fijo el tiempo de espera para el proximo loop
				// Reflejo los cambios de configuracion
				pv_tka_set_waiting_time();
				an_state = anPOLEAR;
			}
			break;
		}
	}

}
//------------------------------------------------------------------------------------
// FUNCIONES PRINCIPALES DE LA TAREA TK_ANALOG
//------------------------------------------------------------------------------------
static void pv_tka_poleo(void)
{

uint16_t adcRetValue;
uint8_t channel;
bool retS;
uint8_t poll_counter;		// Contador de las veces poleadas antes de promediar

// Entry:
	if ( (systemVars.debugLevel & D_DATA) != 0) {
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("%s aDATA::poll:\r\n\0"),u_now() );
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	pv_tka_prender_sensores();
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );	// Espero un settle time de 1s

	// Hago una conversion dummy
	ADC_read( 0, &adcRetValue);
	vTaskDelay( ( TickType_t)( 1500 / portTICK_RATE_MS ) );

	// Init Data Structure
	for ( channel = 0; channel < (NRO_ANALOG_CHANNELS + 1); channel++ )
		rAIn[channel] = 0;

// Loop:
	// Poleo
	poll_counter = CICLOS_POLEO;
	while ( poll_counter-- > 0) {

		for ( channel = 0; channel < (NRO_ANALOG_CHANNELS + 1); channel++ ) {
			adcRetValue = 0;
			retS = ADC_readDlgCh( channel, &adcRetValue);	// AIN0->ADC3; AIN1->ADC5; AIN2->ADC7; BATT->ADC1;
			if ( retS ) {
				rAIn[channel] += adcRetValue;
			} else {

				if ( (systemVars.debugLevel & (D_BASIC + D_DATA) ) != 0) {
					snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("%s aDATA::poll: ERROR: ch_%02d\r\n\0"), u_now(), channel );
					FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
				}

			}

			if ( (systemVars.debugLevel & D_DATA) != 0) {
				snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("%s aDATA::poll: ch_%02d=%d\r\n\0"),u_now(), channel, adcRetValue );
				FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
			}
		}
	}

// Exit:
	pv_tka_apagar_sensores();

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static void pv_tka_save_frame_inBD(void)
{

	// Solo los salvo en la BD si estoy en modo normal.
	// En otros casos ( service, monitor_frame, etc, no.
size_t bytes_written;
StatBuffer_t pxFFStatBuffer;

	if ( (systemVars.debugLevel & D_DATA) != 0) {
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("%s aDATA::bd:\r\n\0"),u_now() );
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	// Guardo en BD
	bytes_written = FF_fwrite( &ANframe, sizeof(ANframe));
	FF_stat(&pxFFStatBuffer);

	if ( bytes_written != sizeof(ANframe) ) {
		// Error de escritura ??
		if ( (systemVars.debugLevel & (D_BASIC + D_DATA) ) != 0) {
			snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("%s aDATA::bd: WR ERROR: (%d)\r\n\0"),u_now(),pxFFStatBuffer.errno);
			FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
		}

	} else {

		// Stats de memoria
		if ( (systemVars.debugLevel & (D_BASIC + D_DATA) ) != 0) {
			snprintf_P( aIn_printfBuff, sizeof(aIn_printfBuff), PSTR("%s aDATA::bd: MEM [%d/%d/%d][%d/%d]\r\n\0"),u_now(), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
			FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
		}
	}

}
//------------------------------------------------------------------------------------
static void pv_tka_signal_tasks(void)
{
	// Indico a otras tareas que hay datos disponibles frescos.

	if ( (systemVars.debugLevel & D_DATA) != 0) {
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("%s aDATA::signal:\r\n\0"),u_now() );
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	// tkOutputs
	while ( xTaskNotify(xHandle_tkOutputs, TK_FRAME_READY , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

	// tkGPRS
	while ( xTaskNotify(xHandle_tkGprsRx, TK_FRAME_READY , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

}
//------------------------------------------------------------------------------------
static void pv_tka_promediar_datos(void)
{
	// Promedio
	// Convierto a magnitud.
	// Completo el frame con fechaHora y datos digitales.

double I,M;
uint16_t D;
uint8_t channel;

	if ( (systemVars.debugLevel & D_DATA) != 0) {
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("%s aDATA::avg:\r\n\0"),u_now() );
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	// Promedio canales analogicos y bateria
	for ( channel = 0; channel < (NRO_ANALOG_CHANNELS + 1); channel++) {
		rAIn[channel] /= CICLOS_POLEO;

		if ( (systemVars.debugLevel & D_DATA) != 0) {
			snprintf_P( aIn_printfBuff,CHAR128,PSTR("%s aDATA::avg: AvgCh[%d]=%.02f\r\n\0"), u_now(), channel, rAIn[channel]);
			FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
		}

	}

	// Convierto los canales analogicos a magnitudes.
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS ; channel++) {

		// Calculo la corriente medida en el canal
		I = rAIn[channel] * systemVars.Imax[channel] / 4096;

		// Calculo la pendiente
		M = 0;
		D = systemVars.Imax[channel] - systemVars.Imin[channel];
		if ( D != 0 ) {
			M = ( systemVars.Mmax[channel]  -  systemVars.Mmin[channel] ) / D;
			rAIn[channel] = systemVars.Mmin[channel] + M * ( I - systemVars.Imin[channel] );
			if ( (systemVars.debugLevel & D_DATA) != 0) {
				snprintf_P( aIn_printfBuff,CHAR128,PSTR("%s aDATA::(ch %d) D=%d, M=%.3f, I=%.3f\r\n\0"), u_now(), channel, D,M,I);
				FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
			}
		} else {
			// Error: denominador = 0.
			rAIn[channel] = -999;
		}

	}

	// Convierto la bateria.
	rAIn[NRO_ANALOG_CHANNELS] = (15 * rAIn[NRO_ANALOG_CHANNELS]) / 4096;	// Bateria

	// DEBUG
	if ( (systemVars.debugLevel & D_DATA) != 0) {
		for ( channel = 0; channel <= NRO_ANALOG_CHANNELS; channel++) {
			snprintf_P( aIn_printfBuff,CHAR128,PSTR("%s aDATA::avg: MagCh[%d]=%.02f\r\n\0"), u_now(), channel, rAIn[channel]);
			FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
		}
	}

	// Armo el frame.
	memset(&ANframe,'\0', sizeof(frameData_t));

	// Agrego la hora
	RTC_read(&ANframe.rtc);

	// Agrego los canales analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		ANframe.analogIn[channel] = rAIn[channel];
	}

	// Agrego la bateria
	ANframe.batt = rAIn[3];

	// Agrego los canales digital ( y reseteo los contadores )
	u_readDigitalCounters( &ANframe.dIn );

}
//------------------------------------------------------------------------------------
static void pv_tka_print_frame(void)
{
	// Imprimo

uint8_t channel;
uint16_t pos = 0;

	if ( (systemVars.debugLevel & (D_BASIC + D_DATA ) ) != 0) {

		// HEADER
		pos = snprintf_P( aIn_printfBuff, sizeof(aIn_printfBuff), PSTR("%s aDATA::frame {" ), u_now() );
		// timeStamp.
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),ANframe.rtc.year,ANframe.rtc.month,ANframe.rtc.day );
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("%02d%02d%02d"),ANframe.rtc.hour,ANframe.rtc.min, ANframe.rtc.sec );

		// Valores analogicos
		for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
			pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR(",%s=%.02f"),systemVars.aChName[channel],ANframe.analogIn[channel] );
		}

		// Valores digitales
		for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
			pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR(",%s=%.02f"), systemVars.dChName[channel],ANframe.dIn.caudal[channel] );

		}

		// Bateria
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR(",bt=%.02f"),ANframe.batt );

		// TAIL
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("}\r\n\0") );

		// Imprimo
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
static void pv_tka_timer_callback( TimerHandle_t pxTimer )
{
	// El timer esta en reload c/1 sec,
	// Ajusto los timers.
	if ( AN_timer > 0 ) {
		-- AN_timer;
	}
}
//------------------------------------------------------------------------------------
static uint16_t pv_tka_set_waiting_time(void)
{

	// Fijo el tiempo de espera para el proximo loop
	// Todos los cambios de configuracion ( pwrModo, timerPoll) se ven reflejados
	// solo en esta funcion.

uint16_t new_wait_time;

	new_wait_time = systemVars.timerPoll;

	// Controlo el limite del tiempo de poleo.
	if ( new_wait_time < 15 ) {
		new_wait_time = 15;
	}

	AN_timer = new_wait_time;

	if ( (systemVars.debugLevel & D_DATA) != 0) {
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("%s aDATA::time: awaiting time %u\r\n\0"), u_now(), new_wait_time );
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	return(new_wait_time);
}
//------------------------------------------------------------------------------------
static void pv_tka_prender_sensores(void)
{

	IO_sensor_pwr_on();
	IO_analog_pwr_on();
	if ( (systemVars.debugLevel & D_DATA) != 0) {
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("%s aDATA::sensors: On\r\n\0"),u_now() );
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
static void pv_tka_apagar_sensores(void)
{

	IO_sensor_pwr_off();
	IO_analog_pwr_off();
	if ( (systemVars.debugLevel & D_DATA) != 0) {
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("%s aDATA::sensors: Off\r\n\0"),u_now() );
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}
}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
int16_t u_readTimeToNextPoll(void)
{

	return (AN_timer);
}
//------------------------------------------------------------------------------------
void u_readDataFrame (frameData_t *dFrame)
{
	memcpy(dFrame, &ANframe, sizeof(ANframe) );
}
//------------------------------------------------------------------------------------
void tkAnalogInit(void)
{
	// Esta funcion se utiliza  antes de arrancar el FRTOS de modo que cree
	// el timer que necesitamos en este modulo
	// Expira c/1sec

	pollingTimer = xTimerCreate (  "POLL_T",
	                     /* The timer period in ticks, must be greater than 0. */
	                     ( 1000 / portTICK_PERIOD_MS) ,
	                     /* The timers will auto-reload themselves when they expire. */
	                     pdTRUE,
	                     /* Assign each timer a unique id equal to its array index. */
	                     ( void * ) NULL,
	                     /* Each timer calls the same callback when it expires. */
						 pv_tka_timer_callback
	                   );

	if ( pollingTimer == NULL )
		u_panic(P_AIN_TIMERCREATE);

	// Inicialmente el timer esta apagado.
	AN_timer = 0;
}
//------------------------------------------------------------------------------------
static void pv_init_AnFrame(void)
{
uint8_t i;

	for ( i = 0 ; i < NRO_ANALOG_CHANNELS; i++ ) {
		ANframe.analogIn[i] = 0;
	}

	for ( i = 0; i < NRO_DIGITAL_CHANNELS; i++ ) {
		ANframe.dIn.caudal[i] = 0;
		ANframe.dIn.pulse_count[i] = 0;
		ANframe.dIn.pulse_period[i] = 0;
		ANframe.dIn.metodo_medida[i] = 'x';
	}

	ANframe.batt = 0;
}
//------------------------------------------------------------------------------------
