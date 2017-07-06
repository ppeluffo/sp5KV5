/*
 * sp5KV5_tkGprs_main.c
 *
 *  Created on: 26 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV5_tkGprs.h"

//-------------------------------------------------------------------------------------
void tkGprsTx(void * pvParameters)
{

( void ) pvParameters;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("starting tkGprsTx..\r\n\0"));
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	memset(buff_gprs_imei,'\0',IMEIBUFFSIZE);

	for( ;; )
	{

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
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("starting tkGprsRX..\r\n\0"));
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	g_flushRXBuffer();

	// loop
	for( ;; )
	{

		// Monitoreo las señales y solo prendo las flags correspondientes.
		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 10 / portTICK_RATE_MS ) );
		if ( xResult == pdTRUE ) {

			if ( ( ulNotifiedValue & TK_PARAM_RELOAD ) != 0 ) {		// Mensaje de reload configuration.
				GPRS_stateVars.signal_reload = true;
			} else if ( ( ulNotifiedValue & TK_REDIAL ) != 0 ) {  	// Mensaje de read frame desde el cmdLine.
				GPRS_stateVars.signal_redial = true;
			} else if ( ( ulNotifiedValue & TK_TILT ) != 0 ) {  	// Mensaje de tilt desde tkControl.
				GPRS_stateVars.signal_tilt = true;
			} else if ( ( ulNotifiedValue & TK_FRAME_READY ) != 0 ) {  	// Mensaje de tilt desde tkControl.
				GPRS_stateVars.signal_frameReady = true;
			}
		}

		// el read se bloquea 50ms. lo que genera la espera.
		while ( FreeRTOS_read( &pdUART0, &c, 1 ) == 1 ) {
			gprsRx.buffer[gprsRx.ptr] = c;
			// Avanzo en modo circular
			gprsRx.ptr = ( gprsRx.ptr  + 1 ) % ( UART0_RXBUFFER_LEN );

			// Los comandos vienen terminados en CR
/*			if (c == '\r') {

				if ( strstr( gprsRx.buffer, "OK") != NULL ) {
					FreeRTOS_write( &pdUART1, "DEBUG ** MRSP_OK\r\n\0", sizeof("DEBUG ** MRSP_OK\r\n\0") );
					// No podemos asumir que el socket este cerrado ya que en las respuestas HTTP puede venir
					// un OK\r.
				}

				if ( strstr( gprsRx.buffer, "ERROR") != NULL ) {
					FreeRTOS_write( &pdUART1, "DEBUG ** MRSP_ERROR\r\n\0", sizeof("DEBUG ** MRSP_ERROR\r\n\0") );
				}

				if ( strstr( gprsRx.buffer, "CONNECT") != NULL ) {
					FreeRTOS_write( &pdUART1, "DEBUG ** MRSP_CONNECT\r\n\0", sizeof("DEBUG ** MRSP_CONNECT\r\n\0") );
				}

				if ( strstr( gprsRx.buffer, "NO CARRIER") != NULL ) {
					FreeRTOS_write( &pdUART1, "DEBUG ** MRSP_NO CARRIER\r\n\0", sizeof("DEBUG ** MRSP_NO CARRIER\r\n\0") );
				}
			}
*/
		}
	}
}
//------------------------------------------------------------------------------------
