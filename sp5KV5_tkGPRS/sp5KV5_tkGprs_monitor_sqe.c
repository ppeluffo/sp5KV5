/*
 * sp5KV5_tkGprs_monitor_sqe.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV5_tkGprs.h"

static void pv_read_sqe(void);
static bool pv_procesar_signals_monsqe( void );

//------------------------------------------------------------------------------------
bool gprs_monitor_sqe(void)
{
	// Me quedo en un loop infinito preguntando por el SQE c/10s y
	// mostrando el resultado.

uint8_t MON_timer = 1;


	GPRS_stateVars.state = G_MON_SQE;

	while ( systemVars.wrkMode == WK_MONITOR_SQE ) {

		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

		// PROCESO LAS SEÑALES
		if ( pv_procesar_signals_monsqe()) {
			// Si recibi alguna senal, debo salir.
			return( bool_RESTART );
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
static bool pv_procesar_signals_monsqe( void )
{
	// Estoy prendiendo el modem de modo que solo me interesa
	// la senal de reload.
	// Las otras en forma implicita las estoy atendiendo

bool ret_f = false;

	if ( GPRS_stateVars.signal_reload) {
		// Salgo a reiniciar tomando los nuevos parametros.
		ret_f = true;
		goto EXIT;
	}

	ret_f = false;

EXIT:

	GPRS_stateVars.signal_reload = false;
	GPRS_stateVars.signal_tilt = false;
	GPRS_stateVars.signal_redial = false;
	GPRS_stateVars.signal_frameReady = false;

	return(ret_f);
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
			snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::monsqe: CSQ=%d,DBM=%d\r\n\0"), u_now(),systemVars.csq,systemVars.dbm );
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}

	}

}
//------------------------------------------------------------------------------------
