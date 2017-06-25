/*
 * sp5KV5_tkGprs_monitor_sqe.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV5_tkGprs.h"

static void pv_read_sqe(void);

//------------------------------------------------------------------------------------
bool gprs_monitor_sqe(void)
{
	// Me quedo en un loop infinito preguntando por el SQE c/10s y
	// mostrando el resultado.

BaseType_t xResult;
uint32_t ulNotifiedValue;
uint8_t MON_timer = 1;


	while ( systemVars.wrkMode == WK_MONITOR_SQE ) {

		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

		// Analizo las señales
		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 250 / portTICK_RATE_MS ) );
		if ( xResult == pdTRUE ) {

			if ( ( ulNotifiedValue & TK_PARAM_RELOAD ) != 0 ) {			// Mensaje de reload configuration.
				return( bool_RESTART );	// Retorna y hace que deba ir a RESTART y leer la nueva configuracion
			} else if ( ( ulNotifiedValue & TK_REDIAL ) != 0 ) {  	// Mensaje de read frame desde el cmdLine.
				return( bool_RESTART );	// Idem
			}
		}

		if ( MON_timer > 0) {	// Espero 1s contando
			MON_timer--;
		} else {
			// Expiro: monitoreo el SQE y recargo el timer.
			pv_read_sqe();
			MON_timer = 10;
		}

	}

	// No estoy en modo mon_sqe: permite salir y continuar el flujo
	return( bool_CONTINUAR );
}
//------------------------------------------------------------------------------------
static void pv_read_sqe(void)
{

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

		if ( (systemVars.debugLevel &  ( D_BASIC + D_GPRS ) ) != 0) {
			snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS_MONSQE:: CSQ=%d,DBM=%d\r\n\0"), u_now(),systemVars.csq,systemVars.dbm );
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}

	}

}
//------------------------------------------------------------------------------------
