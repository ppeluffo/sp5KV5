/*
 * sp5KV5_tkGprs_prender.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV5_tkGprs.h"

static void pv_readImei(void);

// La tarea no puede demorar mas de 180s.
#define WDG_GPRS_TO_PRENDER	180

//------------------------------------------------------------------------------------
bool gprs_prender(void)
{
	// Intento prender el modem hasta 3 veces. Si no puedo, fijo el nuevo tiempo
	// para esperar y salgo.
	// Mientras lo intento prender no atiendo mensajes ( cambio de configuracion / flooding / Redial )

uint8_t hw_tries, sw_tries;
bool exit_flag = bool_RESTART;

// Entry:
	pub_control_watchdog_kick(WDG_GPRS, WDG_GPRS_TO_PRENDER);

	GPRS_stateVars.state = G_PRENDER;
	pub_uarts_ctl(MODEM_PRENDER);

	// Debo poner esta flag en true para que el micro no entre en sleep y pueda funcionar el puerto
	// serial y leer la respuesta del AT del modem.
	GPRS_stateVars.modem_prendido = true;
	vTaskDelay( (portTickType)( 3000 / portTICK_RATE_MS ) );

	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: prender.\r\n\0" ));
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// Me aseguro que el modem este apagado
	IO_modem_hw_pwr_off();
	strncpy_P(systemVars.dlg_ip_address, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;
	vTaskDelay( (portTickType)( 5000 / portTICK_RATE_MS ) );		// Espero 5s

// Loop:
	for ( hw_tries = 0; hw_tries < MAX_HW_TRIES_PWRON; hw_tries++ ) {

		IO_modem_hw_pwr_on();	// Prendo la fuente ( alimento al modem ) HW
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );			// Espero 1s que se estabilize la fuente.

		// Reintento prenderlo activando el switch pin
		for ( sw_tries = 0; sw_tries < MAX_SW_TRIES_PWRON; sw_tries++ ) {

			if ( systemVars.debugLevel ==  D_GPRS ) {
				FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: prender HW=%d,SW=%d\r\n\0"), hw_tries, sw_tries);
				FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
			}

			// Genero el toggle del switch pin para prenderlo
			// Debe estar low por mas de 2s
			IO_modem_sw_switch_high();
			vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
			IO_modem_sw_switch_low();
			vTaskDelay( (portTickType)( 2500 / portTICK_RATE_MS ) );
			IO_modem_sw_switch_high();

			// Espero 10s para interrogarlo
			vTaskDelay( (portTickType)( 10000 / portTICK_RATE_MS ) );

			// Envio un AT y espero un OK para confirmar que prendio.
			pub_gprs_flush_RX_buffer();
			FreeRTOS_write( &pdUART0, "AT\r\0", sizeof("AT\r\0") );
			vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

			pub_gprs_print_RX_Buffer();	// Muestro lo que recibi del modem ( en modo debug )

			// Leo y Evaluo la respuesta al comando AT
			if ( strstr( gprsRx.buffer, "OK") != NULL ) {

				if ( systemVars.debugLevel ==  D_GPRS ) {
					FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Modem prendido\r\n\0" ));
					FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
				}

				// Respondio OK. Esta prendido; salgo
				exit_flag = bool_CONTINUAR;
				goto EXIT;

			} else {

				FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Modem No prendio !!\r\n\0" ));
				FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
			}

			// No prendio: Espero 5s antes de reintentar prenderlo por SW.
			vTaskDelay( (portTickType)( 5000 / portTICK_RATE_MS ) );

		}

		// No prendio luego de MAX_SW_TRIES_PWRON intentos SW. Apago y prendo de nuevo
		IO_modem_hw_pwr_off();	// Apago la fuente
		vTaskDelay( (portTickType)( 10000 / portTICK_RATE_MS ) );			// Espero 10s antes de reintentar
	}

	// Si salgo por aqui es que el modem no prendio luego de todos los reintentos
	exit_flag = bool_RESTART;
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: FAIL!! Modem No prendio en HW%d y SW%d intentos\r\n\0"), MAX_HW_TRIES_PWRON, MAX_SW_TRIES_PWRON);
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// Exit:
EXIT:

	// Ajusto la flag modem_prendido ya que termino el ciclo y el micro pueda entrar en sleep.
	if ( exit_flag == bool_CONTINUAR ) {
		pv_readImei();		// Leo el IMEI
	}

	return(exit_flag);
}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static void pv_readImei(void)
{
	// Leo el imei del modem para poder trasmitirlo al server y asi
	// llevar un control de donde esta c/sim

uint8_t i,j,start, end;

	pub_gprs_flush_RX_buffer();
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT+CGSN\r\0"));
	FreeRTOS_write( &pdUART0, &gprs_printfBuff, sizeof(gprs_printfBuff) );

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	// Leo y Evaluo la respuesta al comando AT+CGSN
	if ( strstr( gprsRx.buffer, "OK") != NULL ) {

		pub_gprs_print_RX_Buffer();

		// Extraigoel IMEI del token. Voy a usar el buffer  de print ya que la respuesta
		// puede ser grande.
		memcpy(gprs_printfBuff, FreeRTOS_UART_getFifoPtr(&pdUART0), sizeof(gprs_printfBuff) );

		// Guardo el IMEI
		start = 0;
		end = 0;
		j = 0;
		// Busco el primer digito
		for ( i = 0; i < 64; i++ ) {
			if ( isdigit( gprs_printfBuff[i]) ) {
				start = i;
				break;
			}
		}
		if ( start == end )		// No lo pude leer.
			goto EXIT;

		// Busco el ultimo digito y copio todos
		for ( i = start; i < IMEIBUFFSIZE; i++ ) {
			if ( isdigit( gprs_printfBuff[i]) ) {
				buff_gprs_imei[j++] = gprs_printfBuff[i];
				buff_gprs_imei[ IMEIBUFFSIZE - 1 ] = '\0';
			} else {
				break;
			}
		}
	}

// Exit
EXIT:

	if ( systemVars.debugLevel == D_GPRS ) {
		FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: IMEI[%s]\r\n\0"), &buff_gprs_imei);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

}
//--------------------------------------------------------------------------------------
