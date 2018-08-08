/*
 * sp5KV5_tkGprs_configurar.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV5_tkGprs.h"

static void pv_gprs_configurar_parametros(void);
static bool pv_gprs_configurar_banda(void);
static bool pv_gprs_net_attach(void);
static void pv_gprs_ask_sqe(void);

// La tarea no puede demorar mas de 180s.
#define WDG_GPRS_TO_CONFIG	180

//------------------------------------------------------------------------------------
bool gprs_configurar(void)
{

	// Configuro los parametros opertativos, la banda GSM y pido una IP de modo que el
	// modem quede listo para
	// No atiendo mensajes ya que no requiero parametros operativos.
	// WATCHDOG: No demoro mas de 2 minutos en este estado

bool exit_flag = bool_RESTART;

// Entry:

	pub_control_watchdog_kick(WDG_GPRS, WDG_GPRS_TO_CONFIG );

	GPRS_stateVars.state = G_CONFIGURAR;

	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: config.\r\n\0" ));
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

// Loop:

	pv_gprs_configurar_parametros();		// Configuro parametros operativos.

	if ( ! pv_gprs_configurar_banda() ) {	// Consiguro la banda: Si necesito resetearme
		exit_flag = bool_RESTART;			// retorna false para salir enseguida
		goto EXIT;
	}

	if ( ! pv_gprs_net_attach() ) {			// Intento conectrme a la red
		exit_flag = bool_RESTART;			// retorna false para salir enseguida
		goto EXIT;
	}

	// Estoy conigurado y conectado a la red: mido el sqe
	pv_gprs_ask_sqe();
	exit_flag = bool_CONTINUAR;

EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static void pv_gprs_configurar_parametros(void)
{

	pub_gprs_flush_RX_buffer();
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT&D0&C1\r\0"));
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	pub_gprs_print_RX_Buffer();

	// Configuro la secuencia de escape +++AT
	pub_gprs_flush_RX_buffer();
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT*E2IPS=2,8,2,1020,1,15\r\0"));
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	pub_gprs_print_RX_Buffer();

	// SMS Envio: Los envio en modo texto
	pub_gprs_flush_RX_buffer();
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT+CMGF=1\r\0"));
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	pub_gprs_print_RX_Buffer();

	// SMS Recepcion: No indico al TE ni le paso el mensaje
	pub_gprs_flush_RX_buffer();
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT+CNMI=1,0\r\0"));
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	pub_gprs_print_RX_Buffer();

	// SMS indicacion: Bajando el RI por 100ms.
	pub_gprs_flush_RX_buffer();
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT*E2SMSRI=100\r\0"));
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	pub_gprs_print_RX_Buffer();

	// Deshabilito los mensajes SMS
	pub_gprs_flush_RX_buffer();
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT*E2IPEV=0,0\r\0"));
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	pub_gprs_print_RX_Buffer();

	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Modem Configurado\r\n\0"));
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

}
//------------------------------------------------------------------------------------
static bool pv_gprs_configurar_banda(void)
{
	// Configuro la banda GSM

char bandBuffer[32];
char *ts = NULL;
uint8_t modemBand;

	// Vemos si la banda configurada es la correcta. Si no la reconfiguro.

	// Leo la banda que tiene el modem configurada
	pub_gprs_flush_RX_buffer();
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT*EBSE?\r\0"));
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

	pub_gprs_print_RX_Buffer();

	// Extraigo de la respuesta la banda
	memcpy(bandBuffer, FreeRTOS_UART_getFifoPtr(&pdUART0), sizeof(bandBuffer) );
	ts = strchr(bandBuffer, ':');
	ts++;
	modemBand = atoi(ts);

	if ( systemVars.debugLevel == D_GPRS ) {
		FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: config mBAND=%d,sBAND=%d\r\n\0"), modemBand, systemVars.gsmBand);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// Analizo
	if ( modemBand != systemVars.gsmBand ) {

		// Reconfiguro.
		pub_gprs_flush_RX_buffer();
		FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff));
		vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

		// Guardo el profile
		pub_gprs_flush_RX_buffer();
		FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT&W\r\0"));
		FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
		vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

		if ( systemVars.debugLevel == D_GPRS ) {
			FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Reconfiguro GSM_BAND a modo %d:\r\n\0"), systemVars.gsmBand);
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}

		// Debo reiniciar el modem para que tome la nueva banda
		return(false);
	}

	if ( systemVars.debugLevel == D_GPRS ) {
		FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Banda GRPS OK.\r\n\0") );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	return(true);
}
//------------------------------------------------------------------------------------
static bool pv_gprs_net_attach(void)
{
	// Doy el comando para atachearme a la red
	// Puede demorar unos segundos por lo que espero para chequear el resultado
	// y reintento varias veces.

uint8_t reintentos = MAX_TRYES_NET_ATTCH;
uint8_t check_tryes;
bool exit_flag = false;

	while ( reintentos-- > 0 ) {

		if ( systemVars.debugLevel == D_GPRS ) {
			FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: NET ATTACH(%d).\r\n\0"),reintentos );
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}

		// Envio el comando
		pub_gprs_flush_RX_buffer();
		FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT+CREG?\r\0"));
		FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

		vTaskDelay( (portTickType)( 5000 / portTICK_RATE_MS ) );	// Espero 5s por la respuesta.

		// Chequeo la respuesta en modo persistente c/2s
		check_tryes = 5;
		while ( check_tryes-- > 0 ) {
			// Leo y Evaluo la respuesta al comando AT+CREG?
			// Sin roaming
			if ( strstr( gprsRx.buffer, "+CREG: 0,1") != NULL ) {
				pub_gprs_print_RX_Buffer();
				// LOG & DEBUG
				if ( systemVars.debugLevel == D_GPRS ) {
					FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: NET OK.\r\n\0" ));
					FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
				}
				exit_flag = true;
				goto EXIT;
			}

			//( roaming !!!. Se usa en Concordia )
			if ( strstr( gprsRx.buffer, "+CREG: 0,5") != NULL ) {
				pub_gprs_print_RX_Buffer();
				// LOG & DEBUG
				if ( systemVars.debugLevel == D_GPRS) {
					FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: NET OK(roaming).\r\n\0" ));
					FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
				}
				exit_flag = true;
				goto EXIT;			}

			vTaskDelay( (portTickType)( 2000 / portTICK_RATE_MS ) );	// Espero 2s mas por la respuesta
			if ( systemVars.debugLevel == D_GPRS ) {
				FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".\0" ));
				FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
			}

		}

		// No pude atachearme. Debo mandar de nuevo el comando
	}

	// Luego de varios reintentos no pude conectarme a la red.
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: NET FAIL !!.\r\n\0" ));
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	exit_flag = false;

	// Exit:
EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
static void pv_gprs_ask_sqe(void)
{
	// Veo la calidad de senal que estoy recibiendo

char csqBuffer[32];
char *ts = NULL;

	// Query SQE
	pub_gprs_flush_RX_buffer();
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT+CSQ\r\0"));
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );
	pub_gprs_print_RX_Buffer();

	memcpy(csqBuffer, FreeRTOS_UART_getFifoPtr(&pdUART0), sizeof(csqBuffer) );
	if ( (ts = strchr(csqBuffer, ':')) ) {
		ts++;
		systemVars.csq = atoi(ts);
		systemVars.dbm = 113 - 2 * systemVars.csq;
	}

	// LOG & DEBUG
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: signalQ CSQ=%d,DBM=%d\r\n\0"),systemVars.csq,systemVars.dbm );
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

}
//------------------------------------------------------------------------------------
