/*
 * sp5KV3_tkAnalogIn.c
 *
 *  Created on: 14/4/2015
 *      Author: pablo
 */

#include <sp5KV5.h>

#define CICLOS_POLEO	20		// ciclos de poleo para promediar.

static char aIn_printfBuff[CHAR256];
static double rAIn[NRO_ANALOG_CHANNELS + 1];	// Almaceno los datos de conversor A/D
static double adc_samples[NRO_ANALOG_CHANNELS + 1][CICLOS_POLEO];

static frameData_t pv_data_frame;

// FUNCIONES PRIVADAS
static void pv_analog_prender_sensores(void);
static void pv_analog_apagar_sensores(void);
static void pv_analog_read_adc ( void );
static void pv_analog_convert_raw_to_mag(void);
static bool pv_analog_save_frame_inBD(void);

// La tarea pasa por el mismo lugar c/timerPoll secs.
#define WDG_AIN_TIMEOUT	( systemVars.timerPoll + 60 )

//-------------------------------------------------------------------------------------
void tkAnalogIn(void * pvParameters)
{

( void ) pvParameters;
uint32_t waiting_ticks;
TickType_t xLastWakeTime;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	FRTOS_snprintf( aIn_printfBuff,sizeof(aIn_printfBuff),"starting tkAnalogIn..\r\n\0");
	FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    // Al arrancar poleo a los 5s
    waiting_ticks = (uint32_t)(5) * 1000 / portTICK_RATE_MS;

	pv_analog_apagar_sensores(); 	// Los sensores arrancan apagados

	// loop
	for( ;; )
	{

    	pub_control_watchdog_kick(WDG_AIN, WDG_AIN_TIMEOUT);

		vTaskDelayUntil( &xLastWakeTime, waiting_ticks ); // Da el tiempo para entrar en tickless.

		// Leo analog,digital,rtc,salvo en BD e imprimo.
		pub_analog_read_frame(true);

		// Espero un ciclo
		while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
			taskYIELD();
		waiting_ticks = (uint32_t)(systemVars.timerPoll) * 1000 / portTICK_RATE_MS;
		xSemaphoreGive( sem_SYSVars );

	}

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
void pub_analog_read_frame(bool saveInBD )
{
	// Funcion usada para leer los datos de todos los modulos, guardarlos en memoria
	// e imprimirlos.
	// La usa por un lado tkData en forma periodica y desde el cmd line cuando se
	// da el comando read frame.

static bool primer_frame = true;
bool retS = false;
uint8_t channel;

	// Leo los canales analogicos.
	pv_analog_read_adc();

	// Convierto los valores a magnitudes
	pv_analog_convert_raw_to_mag();

	// Armo el frame.
	memset(&pv_data_frame,'\0', sizeof(frameData_t));

	// Agrego el timestamp
	RTC_read(&pv_data_frame.rtc);

	// Agrego los canales analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		pv_data_frame.analogIn[channel] = rAIn[channel];
	}

	// Agrego la bateria
	pv_data_frame.batt = rAIn[3];

	// Agrego los canales digital ( y reseteo los contadores )
	pub_digital_read_counters( &pv_data_frame.dIn );

	// Agrego el canal digital de nivel 0.
	pv_data_frame.dIn.level0 = IO_read_dinL0_pin();

	// Para no incorporar el error de los contadores en el primer frame no lo guardo.
	if ( primer_frame ) {
		primer_frame = false;
		return;
	}

	// Si me invocaron por modo comando, no salvo en BD
	if ( saveInBD ) {
		// Modo normal: salvo en la BD
		retS = pv_analog_save_frame_inBD();
		if ( !retS ) {
			FRTOS_snprintf( aIn_printfBuff, sizeof(aIn_printfBuff) , "Data BD save ERROR !!\r\n\0");
			FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
			return;
		}
		// y aviso a tkGprs que hay un frame listo
		while ( xTaskNotify(xHandle_tkGprsRx, TK_FRAME_READY , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
	}

	// Muestro en pantalla.
	pub_analog_print_frame(&pv_data_frame);


}
//------------------------------------------------------------------------------------
static void pv_analog_read_adc ( void )
{
	// Realiza  el poleo de los canales del conversor A/D.

uint16_t adcRetValue;
uint8_t channel;
bool retS;
uint8_t items;

// Entry:
	if ( systemVars.debugLevel == D_DATA)  {
		FRTOS_snprintf( aIn_printfBuff,sizeof(aIn_printfBuff),"DATA: poll\r\n\0" );
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	pv_analog_prender_sensores();
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );	// Espero un settle time de 1s

	// Hago una conversion dummy
	ADC_read( 0, &adcRetValue);
	vTaskDelay( ( TickType_t)( 1500 / portTICK_RATE_MS ) );

	// Init Data Structure
	for (channel = 0; channel < (NRO_ANALOG_CHANNELS + 1); channel++ )
		for (items = 0; items < CICLOS_POLEO; items++ )
			adc_samples[channel][items] = 0.0;

	// Poleo
	for ( items = 0; items < CICLOS_POLEO; items++)
	{
		for ( channel = 0; channel < (NRO_ANALOG_CHANNELS + 1); channel++ )
		{
			adcRetValue = 0;
			retS = ADC_readDlgCh( channel, &adcRetValue);	// AIN0->ADC3; AIN1->ADC5; AIN2->ADC7; BATT->ADC1;

			if ( retS ) {
				adc_samples[channel][items] = adcRetValue;
				if ( systemVars.debugLevel == D_DATA ) {
					FRTOS_snprintf( aIn_printfBuff,sizeof(aIn_printfBuff),"DATA: poll ch_%02d=%d\r\n\0", channel, adcRetValue );
					FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
				}

			} else {
				FRTOS_snprintf( aIn_printfBuff,sizeof(aIn_printfBuff),"DATA: poll ERROR: ch_%02d\r\n\0", channel );
				FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
			}
		}

		vTaskDelay( ( TickType_t)( 250 / portTICK_RATE_MS ) );	// Espero 250ms. f = 4hz.
	}

// Exit:
	pv_analog_apagar_sensores();

}
//------------------------------------------------------------------------------------
static void pv_analog_convert_raw_to_mag(void)
{
	// Convierte los valores del conversor A/D a magnitudes.
	// Promedia, calcula la varianza para descartar outliners y vuelve
	// a promediar con los valores dentro del rango permitido.
	// Luego convierte a magnitud.

double I,M;
uint16_t D;
uint8_t channel;
double avg[NRO_ANALOG_CHANNELS + 1];
double sigma[NRO_ANALOG_CHANNELS + 1];
uint8_t i;
uint8_t count;

	// Calculo el promedio y varianza de los 4 canales;
	for ( channel = 0; channel < ( NRO_ANALOG_CHANNELS + 1); channel++ ) {
		avg[channel] = 0.0;
		sigma[channel] = 0.0;
		for (i=0; i < CICLOS_POLEO; i++) {
			avg[channel] += adc_samples[channel][i];
			sigma[channel] += ( adc_samples[channel][i] * adc_samples[channel][i] );
		}
		avg[channel] /= CICLOS_POLEO;
		sigma[channel] = sqrt (sigma[channel] / CICLOS_POLEO -  ( avg[channel] * avg[channel] ));
		if ( systemVars.debugLevel == D_DATA ) {
			FRTOS_snprintf( aIn_printfBuff,CHAR128,"DATA :avg: AvgCh[%d]=%.02f, Sg=%.02f \r\n\0", channel, avg[channel], sigma[channel]);
			FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
		}
	}

	// Calculo los nuevos promedios con aquellos valores que caen entre avg y 1 sigma.
	for ( channel = 0; channel < ( NRO_ANALOG_CHANNELS + 1); channel++ ) {
		rAIn[channel] = 0.0;
		count = 0;
		for ( i = 0; i < CICLOS_POLEO; i++ ) {
			if ( abs ( adc_samples[channel][i] - avg[channel]) < sigma[channel] ) {
				count++;
				rAIn[channel] += adc_samples[channel][i];
			}
		}
		rAIn[channel] /= count;
		if ( systemVars.debugLevel == D_DATA ) {
			FRTOS_snprintf( aIn_printfBuff,CHAR128,"DATA :AvgOptimo[%d]=%.02f \r\n\0", channel, rAIn[channel]);
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
			if ( systemVars.debugLevel ==  D_DATA)  {
				FRTOS_snprintf( aIn_printfBuff,CHAR128,"DATA:(ch %d) D=%d, M=%.3f, I=%.3f, Mag=%.02f\r\n\0",channel, D,M,I, rAIn[channel]);
				FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
			}
		} else {
			// Error: denominador = 0.
			rAIn[channel] = -999;
		}

	}

	// Convierto la bateria.
	rAIn[NRO_ANALOG_CHANNELS] = (15 * rAIn[NRO_ANALOG_CHANNELS]) / 4096;	// Bateria
	if ( systemVars.debugLevel == D_DATA) {
		FRTOS_snprintf( aIn_printfBuff,CHAR128,"DATA: batt=%.02f\r\n\0", rAIn[NRO_ANALOG_CHANNELS]);
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
static bool pv_analog_save_frame_inBD(void)
{

size_t bytes_written;
StatBuffer_t pxFFStatBuffer;

	// Guardo en BD
	bytes_written = FF_fwrite( &pv_data_frame, sizeof(pv_data_frame));
	FF_stat(&pxFFStatBuffer);

	if ( bytes_written != sizeof(pv_data_frame) ) {
		// Error de escritura ??
		FRTOS_snprintf( aIn_printfBuff,sizeof(aIn_printfBuff),"DATA: WR ERROR: (%d)\r\n\0",pxFFStatBuffer.errno);
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
		return(false);

	} else {

		// Stats de memoria
		FRTOS_snprintf( aIn_printfBuff, sizeof(aIn_printfBuff), "DATA: MEM [%d/%d/%d][%d/%d]--", pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}
	return(true);
}
//------------------------------------------------------------------------------------
void pub_analog_print_frame(frameData_t *dframe)
{
	// Imprimo

uint8_t channel;
uint16_t pos = 0;

	// HEADER
	pos = FRTOS_snprintf( aIn_printfBuff, sizeof(aIn_printfBuff), "frame {" );
	// timeStamp.
	pos += FRTOS_snprintf( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ),"%04d%02d%02d,",dframe->rtc.year,dframe->rtc.month,dframe->rtc.day );
	pos += FRTOS_snprintf( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ),"%02d%02d%02d",dframe->rtc.hour,dframe->rtc.min, dframe->rtc.sec );

	// Valores analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		pos += FRTOS_snprintf( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), ",%s=%.02f",systemVars.aChName[channel],dframe->analogIn[channel] );
	}

	// Valores digitales
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		pos += FRTOS_snprintf( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), ",%s=%.02f", systemVars.dChName[channel],dframe->dIn.caudal[channel] );
	}

	// Nivel digital del canal 0.
	pos += FRTOS_snprintf( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), ",L0=%d", dframe->dIn.level0 );

	// Bateria
	pos += FRTOS_snprintf( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), ",bt=%.02f",dframe->batt );

	// TAIL
	pos += FRTOS_snprintf( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), "}\r\n\0" );

	// Imprimo
	FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );

}
//------------------------------------------------------------------------------------
static void pv_analog_prender_sensores(void)
{

	IO_sensor_pwr_on();
	IO_analog_pwr_on();
	if ( systemVars.debugLevel == D_DATA) {
		FRTOS_snprintf( aIn_printfBuff,sizeof(aIn_printfBuff),"DATA: sensors On\r\n\0");
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
static void pv_analog_apagar_sensores(void)
{

	IO_sensor_pwr_off();
	IO_analog_pwr_off();
	if ( systemVars.debugLevel == D_DATA) {
		FRTOS_snprintf( aIn_printfBuff,sizeof(aIn_printfBuff),"DATA: sensors Off\r\n\0");
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}
}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
bool pub_analog_config_timerpoll(char *s_tPoll)
{
	// Configura el tiempo de poleo.
	// El cambio puede ser desde tkCmd o tkGprs(init frame)
	// Le avisa a la tarea tkAnalog del cambio

uint16_t tpoll;

	tpoll = abs((uint16_t) ( atol(s_tPoll) ));
	if ( tpoll < 15 ) { tpoll = 15; }

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.timerPoll = tpoll;
	xSemaphoreGive( sem_SYSVars );

	return(true);
}
//------------------------------------------------------------------------------------
bool pub_analog_config_channel( uint8_t channel, char *chName, char *s_iMin, char *s_iMax, char *s_mMin, char *s_mMax )
{
	// p1 = name, p2 = iMin, p3 = iMax, p4 = mMin, p5 = mMax

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	if ( chName != NULL ) {
		memset ( systemVars.aChName[channel], '\0',   PARAMNAME_LENGTH );
		memcpy( systemVars.aChName[channel], chName , ( PARAMNAME_LENGTH - 1 ));
	}

	if ( s_iMin != NULL ) { systemVars.Imin[channel] = atoi(s_iMin); }
	if ( s_iMax != NULL ) {	systemVars.Imax[channel] = atoi(s_iMax); }
	if ( s_mMin != NULL ) {	systemVars.Mmin[channel] = atoi(s_mMin); }
	if ( s_mMax != NULL ) {	systemVars.Mmax[channel] = atof(s_mMax); }

	xSemaphoreGive( sem_SYSVars );

	return(true);

}
//----------------------------------------------------------------------------------------
void pub_analog_load_defaults(void)
{

	// Realiza la configuracion por defecto de los canales analogicos.

uint8_t channel;

	systemVars.timerPoll = 300;			// Poleo c/5 minutos

	// Todos los canales quedan por default en 0-20mA, 0-6k.
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS ; channel++) {
		systemVars.Imin[channel] = 0;
		systemVars.Imax[channel] = 20;
		systemVars.Mmin[channel] = 0;
		systemVars.Mmax[channel] = 6.0;
	}

	strncpy_P(systemVars.aChName[0], PSTR("pA\0"),3);
	strncpy_P(systemVars.aChName[1], PSTR("pB\0"),3);
	strncpy_P(systemVars.aChName[2], PSTR("pC\0"),3);

}
//------------------------------------------------------------------------------------
frameData_t *pub_analog_get_data_frame_ptr(void)
{
	return(&pv_data_frame);
}
//------------------------------------------------------------------------------------
