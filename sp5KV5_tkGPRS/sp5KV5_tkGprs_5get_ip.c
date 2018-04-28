/*
 * sp5KV5_tkGprs_ask_ip.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV5_tkGprs.h"

static void pv_read_ip_assigned(void);

// La tarea no puede demorar mas de 180s.
#define WDG_GPRS_TO_IP	180

//------------------------------------------------------------------------------------
bool gprs_get_ip(void)
{
	// El modem esta prendido y configurado.
	// Intento hasta 3 veces pedir la IP.
	// WATCHDOG: En el peor caso demoro 2 mins.

uint8_t ip_queries;
uint8_t check_tryes;
bool exit_flag = bool_RESTART;

	pub_control_watchdog_kick(WDG_GPRS, WDG_GPRS_TO_IP );

// Entry:
	GPRS_stateVars.state = G_GET_IP;

	// APN
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: SET APN\r\n\0" ));
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	pub_gprs_flush_RX_buffer();

	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT+CGDCONT=1,\"IP\",\"%s\"\r\0"),systemVars.apn);
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	pub_gprs_print_RX_Buffer();

	// Intento MAX_IP_QUERIES veces que me asignen una IP.
	for ( ip_queries = 0; ip_queries < MAX_IP_QUERIES; ip_queries++ ) {

		if ( systemVars.debugLevel == D_GPRS ) {
			FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: ask IP(%d):\r\n\0"),ip_queries);
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}

		// Envio el comando AT*E2IPA
		pub_gprs_flush_RX_buffer();
		FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT*E2IPA=1,1\r\0"),systemVars.apn);
		FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

		// Chequeo la respuesta en modo persistente 10 veces c/3s
		check_tryes = 10;
		while ( check_tryes-- > 0 ) {

			// Doy tiempo a que responda la red
			g_sleep(3);

			// Analizo la respuesta
			if ( strstr( gprsRx.buffer, "E2IPA: 000") != NULL ) {
				pub_gprs_print_RX_Buffer();
				exit_flag = bool_CONTINUAR;
				if ( systemVars.debugLevel == D_GPRS ) {
					FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: IP OK.\r\n\0" ));
					FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
				}
				pv_read_ip_assigned();	// Leo e informo cual IP me asigno la red
				goto EXIT;
			}

			if ( strstr( gprsRx.buffer, "ERROR") != NULL ) {
				pub_gprs_print_RX_Buffer();
				// Error: salgo del loop de espera y voy a reintentar dar el comando
				break;
			}

		}

		// Espero 5s antes de dar el comando AT de nuevo
		g_sleep(5);
	}

	// Aqui es que luego de tantos reintentos no consegui la IP.
	exit_flag = bool_RESTART;
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: ip FAIL !!.\r\n\0" ));
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

// Exit
EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static void pv_read_ip_assigned(void)
{
	// Tengo la IP asignada: la leo para actualizar systemVars.ipaddress

char *ts = NULL;
int i=0;
char c;

	// Envio el comado AT*E2IPI para leer la IP
	pub_gprs_flush_RX_buffer();
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT*E2IPA=1,1\r\0"),systemVars.apn);
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT*E2IPI=0\r\0"));
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );

	//  Muestro la IP en pantalla
	pub_gprs_print_RX_Buffer();

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
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: IP=[%s]\r\n\0"), systemVars.dlg_ip_address);
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

}
//------------------------------------------------------------------------------------
