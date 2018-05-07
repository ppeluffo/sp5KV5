/*
 * sp5KV5_tkGprs_main.c
 *
 *  Created on: 26 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV5_tkGprs.h"

#define WDG_GPRSRX_TO	60
//-------------------------------------------------------------------------------------
void tkGprsTx(void * pvParameters)
{

( void ) pvParameters;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("starting tkGprsTx..\r\n\0"));
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	memset(buff_gprs_imei,'\0',IMEIBUFFSIZE);

	for( ;; )
	{

		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
RESTART:

		if ( gprs_esperar_apagado() != bool_CONTINUAR ) {	// Espero con el modem apagado
			goto RESTART;
		}

		if ( gprs_prender() != bool_CONTINUAR )  {	// Intento prenderlo y si no salgo con false
			goto RESTART;			// y voy directo a APAGARLO
		}

		if ( gprs_configurar() != bool_CONTINUAR ) {	// Intento configurarlo y si no puedo o debo reiniciarlo ( cambio de
			goto RESTART;				// banda ) salgo con false y vuelvo a APAGAR
		}

		if ( gprs_monitor_sqe() != bool_CONTINUAR ) {	// Salgo por signal o reset o timeout de control task
			goto RESTART;
		}

		if ( gprs_get_ip() != bool_CONTINUAR  ) {	// Si no logro una IP debo reiniciarme. Salgo en este caso
			goto RESTART;							// con false
		}

		if ( ! gprs_init_frame() ) {	// Si no pude enviar exitosamente el INIT vuelvo a APAGAR.
			goto RESTART;
		}

		if ( gprs_data() != bool_CONTINUAR ) {		// Trasmito datos si hay. En modo DISCRETO termino y vuelvo a apagarme y esperar
			goto RESTART;							// En modo CONTINUO me quedo en esperando datos y trasmitiendo
		}
	}
}

//------------------------------------------------------------------------------------
void tkGprsRx(void * pvParameters)
{
	// Esta tarea lee y procesa las respuestas del GPRS. Lee c/caracter recibido y lo va
	// metiendo en un buffer circular

( void ) pvParameters;
BaseType_t xResult;
uint32_t ulNotifiedValue;
char c;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 200 / portTICK_RATE_MS ) );

	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("starting tkGprsRX..\r\n\0"));
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	pub_gprs_flush_RX_buffer();

	// loop
	for( ;; )
	{

		pub_control_watchdog_kick( WDG_GPRSRX, WDG_GPRSRX_TO );

		// Monitoreo las señales y solo prendo las flags correspondientes.
		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 10 / portTICK_RATE_MS ) );

		if ( xResult == pdTRUE ) {

			if ( ( ulNotifiedValue & TK_REDIAL ) != 0 ) {
				GPRS_stateVars.signal_redial = true;
			} else if ( ( ulNotifiedValue & TK_FRAME_READY ) != 0 ) {
				GPRS_stateVars.signal_frameReady = true;
			}
		}

		// el read se bloquea 50ms. lo que genera la espera.
		while ( FreeRTOS_read( &pdUART0, &c, 1 ) == 1 ) {
			gprsRx.buffer[gprsRx.ptr] = c;
			// Avanzo en modo circular
			gprsRx.ptr = ( gprsRx.ptr  + 1 ) % ( UART0_RXBUFFER_LEN );

		}
	}
}
//------------------------------------------------------------------------------------
