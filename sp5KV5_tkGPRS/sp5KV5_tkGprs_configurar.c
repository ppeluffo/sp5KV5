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

//------------------------------------------------------------------------------------
bool gprs_configurar(void)
{

	// Configuro los parametros opertativos, la banda GSM y pido una IP de modo que el
	// modem quede listo para
	// No atiendo mensajes ya que no requiero parametros operativos.
	// WATCHDOG: No demoro mas de 2 minutos en este estado

bool exit_flag = bool_RESTART;

// Entry:

	GPRS_stateVars.state = G_CONFIGURAR;

	if ( (systemVars.debugLevel & ( D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::config:\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

// Loop:

	// PROCESO LAS SEÑALES
	// No hay loops: no las proceso

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

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT&D0&C1\r\0", sizeof("AT&D0&C1\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// Configuro la secuencia de escape +++AT
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT*E2IPS=2,8,2,1020,1,15\r\0", sizeof("AT*E2IPS=2,8,2,1020,1,15\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// SMS Envio: Los envio en modo texto
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT+CMGF=1\r\0", sizeof("AT+CMGF=1\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// SMS Recepcion: No indico al TE ni le paso el mensaje
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT+CNMI=1,0\r\0", sizeof("AT+CNMI=1,0\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// SMS indicacion: Bajando el RI por 100ms.
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT*E2SMSRI=100\r\0", sizeof("AT*E2SMSRI=100\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// Deshabilito los mensajes SMS
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT*E2IPEV=0,0\r\0", sizeof("AT*E2IPEV=0,0\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	if ( (systemVars.debugLevel & ( D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::config: Modem Configurado\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

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
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT*EBSE?\r\0", sizeof("AT*EBSE?\r\0") );
	g_sleep(1);

	g_printRxBuffer();

	// Extraigo de la respuesta la banda
	memcpy(bandBuffer, FreeRTOS_UART_getFifoPtr(&pdUART0), sizeof(bandBuffer) );
	ts = strchr(bandBuffer, ':');
	ts++;
	modemBand = atoi(ts);

	if ( (systemVars.debugLevel & ( D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::config: mBAND=%d,sBAND=%d\r\n\0"),u_now(), modemBand, systemVars.gsmBand);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// Analizo
	if ( modemBand != systemVars.gsmBand ) {

		// Reconfiguro.
		snprintf_P( gprs_printfBuff,CHAR256,PSTR("AT*EBSE=%d\r\0"),systemVars.gsmBand );
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
		g_flushRXBuffer();
		FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff));
		vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

		// Guardo el profile
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
		g_flushRXBuffer();
		FreeRTOS_write( &pdUART0, "AT&W\r\0", sizeof("AT&W\r\0") );
		vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

		if ( (systemVars.debugLevel & ( D_GPRS) ) != 0) {
			snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::config: Reconfiguro GSM_BAND a modo %d:\r\n\0"), u_now(),systemVars.gsmBand);
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}

		// Debo reiniciar el modem para que tome la nueva banda
		return(false);
	}

	if ( (systemVars.debugLevel & ( D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::config: Banda GRPS OK.\r\n\0"), u_now() );
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

	if ( (systemVars.debugLevel & ( D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::config: NET ATTACH.\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	while ( reintentos-- > 0 ) {

		// Envio el comando
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
		g_flushRXBuffer();
		FreeRTOS_write( &pdUART0, "AT+CREG?\r\0", sizeof("AT+CREG?\r\0") );

		g_sleep(5);	// Espero 5s por la respuesta.

		// Chequeo la respuesta en modo persistente c/2s
		check_tryes = 5;
		while ( check_tryes-- > 0 ) {
			// Leo y Evaluo la respuesta al comando AT+CREG?
			// Sin roaming
			if ( strstr( gprsRx.buffer, "+CREG: 0,1") != NULL ) {
				g_printRxBuffer();
				// LOG & DEBUG
				if ( (systemVars.debugLevel & ( D_BASIC + D_GPRS) ) != 0) {
					snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::config: NET OK.\r\n\0"), u_now() );
					FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
				}
				exit_flag = true;
				goto EXIT;
			}

			//( roaming !!!. Se usa en Concordia )
			if ( strstr( gprsRx.buffer, "+CREG: 0,5") != NULL ) {
				g_printRxBuffer();
				// LOG & DEBUG
				if ( (systemVars.debugLevel & ( D_BASIC + D_GPRS) ) != 0) {
					snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::config: NET OK(roaming).\r\n\0"), u_now() );
					FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
				}
				exit_flag = true;
				goto EXIT;			}

			g_sleep(2);	// Espero 2s mas por la respuesta
		}

		// No pude atachearme. Debo mandar de nuevo el comando
	}

	// Luego de varios reintentos no pude conectarme a la red.
	if ( (systemVars.debugLevel & ( D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::config: NET FAIL !!.\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}
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
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT+CSQ\r\0", sizeof("AT+CSQ\r\0") );

	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	memcpy(csqBuffer, FreeRTOS_UART_getFifoPtr(&pdUART0), sizeof(csqBuffer) );
	if ( (ts = strchr(csqBuffer, ':')) ) {
		ts++;
		systemVars.csq = atoi(ts);
		systemVars.dbm = 113 - 2 * systemVars.csq;
	}

	// LOG & DEBUG
	if ( (systemVars.debugLevel & ( D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::config: signalQ CSQ=%d,DBM=%d\r\n\0"), u_now(),systemVars.csq,systemVars.dbm );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
