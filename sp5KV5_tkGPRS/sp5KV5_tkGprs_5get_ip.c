/*
 * sp5KV5_tkGprs_ask_ip.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV5_tkGprs.h"

static void pv_read_ip_assigned(void);
static bool pv_procesar_signals_getip( bool *exit_flag );
//------------------------------------------------------------------------------------
bool gprs_get_ip(void)
{
	// El modem esta prendido y configurado.
	// Intento hasta 3 veces pedir la IP.
	// WATCHDOG: En el peor caso demoro 2 mins.

uint8_t ip_queries;
uint8_t check_tryes;
bool exit_flag = bool_RESTART;

// Entry:
	GPRS_stateVars.state = G_GET_IP;

	// APN
	FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"GPRS: SET APN\r\n\0" );
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
	g_flushRXBuffer();

	FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"AT+CGDCONT=1,\"IP\",\"%s\"\r\0",systemVars.apn);
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// Intento MAX_IP_QUERIES veces que me asignen una IP.
	for ( ip_queries = 0; ip_queries < MAX_IP_QUERIES; ip_queries++ ) {

		if ( systemVars.debugLevel == D_GPRS ) {
			FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"GPRS: ask IP(%d):\r\n\0",ip_queries);
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}

		// Envio el comando AT*E2IPA
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
		g_flushRXBuffer();

		FreeRTOS_write( &pdUART0, "AT*E2IPA=1,1\r\0", sizeof("AT*E2IPA=1,1\r\0") );

		// Chequeo la respuesta en modo persistente 10 veces c/3s
		check_tryes = 10;
		while ( check_tryes-- > 0 ) {

			// PROCESO LAS SEÑALES
			if ( pv_procesar_signals_getip( &exit_flag )) {
				// Si recibi alguna senal, debo salir.
				goto EXIT;
			}

			// Doy tiempo a que responda la red
			g_sleep(3);

			// Analizo la respuesta
			if ( strstr( gprsRx.buffer, "E2IPA: 000") != NULL ) {
				g_printRxBuffer();
				exit_flag = bool_CONTINUAR;
				if ( systemVars.debugLevel == D_GPRS ) {
					FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"GPRS: IP OK.\r\n\0" );
					FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
				}
				pv_read_ip_assigned();	// Leo e informo cual IP me asigno la red
				goto EXIT;
			}

			if ( strstr( gprsRx.buffer, "ERROR") != NULL ) {
				g_printRxBuffer();
				// Error: salgo del loop de espera y voy a reintentar dar el comando
				break;
			}

		}

		// Espero 5s antes de dar el comando AT de nuevo
		g_sleep(5);
	}

	// Aqui es que luego de tantos reintentos no consegui la IP.
	exit_flag = bool_RESTART;
	FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"GPRS: ip FAIL !!.\r\n\0" );
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

// Exit
EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static bool pv_procesar_signals_getip( bool *exit_flag )
{
	// Estoy prendiendo el modem de modo que solo me interesa
	// la senal de reload.
	// Las otras en forma implicita las estoy atendiendo

bool ret_f = false;

EXIT:

	GPRS_stateVars.signal_redial = false;
	GPRS_stateVars.signal_frameReady = false;

	return(ret_f);
}
//------------------------------------------------------------------------------------
static void pv_read_ip_assigned(void)
{
	// Tengo la IP asignada: la leo para actualizar systemVars.ipaddress

char *ts = NULL;
int i=0;
char c;

	// Envio el comado AT*E2IPI para leer la IP
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT*E2IPI=0\r\0", sizeof("AT*E2IPI=0\r\0") );
	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );

	//  Muestro la IP en pantalla
	g_printRxBuffer();

	// Extraigo la IP del token. Voy a usar el buffer  de print ya que la respuesta
	// puede ser grande.
	memcpy(gprs_printfBuff, FreeRTOS_UART_getFifoPtr(&pdUART0), sizeof(gprs_printfBuff) );

	ts = strchr( gprs_printfBuff, '\"');
	ts++;
	while ( (c= *ts) != '\"') {
		systemVars.dlg_ip_address[i++] = c;
		ts++;
	}
	systemVars.dlg_ip_address[i++] = '\0';
	FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"GPRS: IP=[%s]\r\n\0", systemVars.dlg_ip_address);
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

}
//------------------------------------------------------------------------------------
