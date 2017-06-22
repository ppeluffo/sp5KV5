/*
 * sp5KV5_tkGprs_monitor_sqe.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV5_tkGprs.h"
//------------------------------------------------------------------------------------
void gprs_monitor_sqe(void)
{
	// Me quedo en un loop infinito preguntando por el SQE c/10s y
	// mostrando el resultado.
	// Salgo por reset
	// Como el modo monitor sqe se corre por cmdline no importa el watchdog
	// ya que siempre hay un operador en el equipo.
	// De todos modos debo implementarlo para que el dlg no se resetee.


char csqBuffer[32];
char *ts = NULL;

	for (;;) {

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

		pv_gprs_sleep(15);
	}

EXIT:

	return;

}
//------------------------------------------------------------------------------------
