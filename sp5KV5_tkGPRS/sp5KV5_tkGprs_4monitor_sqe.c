/*
 * sp5KV5_tkGprs_monitor_sqe.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV5_tkGprs.h"

static void pv_read_sqe(void);

// Estoy en modo comando por lo que no importa tanto el wdg.
// Le doy 15 minutos
#define WDG_GPRS_TO_SQE	900

//------------------------------------------------------------------------------------
bool gprs_monitor_sqe(void)
{
	// Me quedo en un loop infinito preguntando por el SQE c/10s y
	// mostrando el resultado.

uint8_t MON_timer = 1;


	GPRS_stateVars.state = G_MON_SQE;

	pub_control_watchdog_kick(WDG_GPRS, WDG_GPRS_TO_SQE );

	while ( systemVars.wrkMode == WK_MONITOR_SQE ) {

		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

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

		if ( systemVars.debugLevel == D_GPRS ) {
			FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: CSQ=%d,DBM=%d\r\n\0"),systemVars.csq,systemVars.dbm );
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}

	}

}
//------------------------------------------------------------------------------------
