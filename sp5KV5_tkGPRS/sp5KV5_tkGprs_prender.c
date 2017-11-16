/*
 * sp5KV5_tkGprs_prender.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV5_tkGprs.h"

static void pv_readImei(void);
static bool pv_procesar_signals_prender( bool *exit_flag );

//------------------------------------------------------------------------------------
bool gprs_prender(void)
{
	// Intento prender el modem hasta 3 veces. Si no puedo, fijo el nuevo tiempo
	// para esperar y salgo.
	// Mientras lo intento prender no atiendo mensajes ( cambio de configuracion / flooding / Redial )

uint8_t hw_tries, sw_tries;
bool exit_flag = bool_RESTART;

// Entry:

	GPRS_stateVars.state = G_PRENDER;
	u_uarts_ctl(MODEM_PRENDER);

	// Debo poner esta flag en true para que el micro no entre en sleep y pueda funcionar el puerto
	// serial y leer la respuesta del AT del modem.
	GPRS_stateVars.modem_prendido = true;
	g_sleep(3);

	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::prender:\r\n\0"), u_now());
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// Me aseguro que el modem este apagado
	IO_modem_hw_pwr_off();
	strncpy_P(systemVars.dlg_ip_address, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;
	g_sleep(5);		// Espero 5s

// Loop:
	for ( hw_tries = 0; hw_tries < MAX_HW_TRIES_PWRON; hw_tries++ ) {

		IO_modem_hw_pwr_on();	// Prendo la fuente ( alimento al modem ) HW
		g_sleep(1);				// Espero 1s que se estabilize la fuente.

		// Reintento prenderlo activando el switch pin
		for ( sw_tries = 0; sw_tries < MAX_SW_TRIES_PWRON; sw_tries++ ) {


			if ( (systemVars.debugLevel &  D_GPRS ) != 0) {
				snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::prender: HW=%d,SW=%d\r\n\0"), u_now(), hw_tries, sw_tries);
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
			g_sleep(10);

			// Envio un AT y espero un OK para confirmar que prendio.
			FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
			FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
			g_flushRXBuffer();
			FreeRTOS_write( &pdUART0, "AT\r\0", sizeof("AT\r\0") );
			g_sleep(1);

			g_printRxBuffer();	// Muestro lo que recibi del modem ( en modo debug )

			// Leo y Evaluo la respuesta al comando AT
			if ( strstr( gprsRx.buffer, "OK") != NULL ) {

				if ( (systemVars.debugLevel &  D_GPRS ) != 0) {
					snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::prender: Modem prendido\r\n\0"), u_now());
					FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
				}

				// Respondio OK. Esta prendido; salgo
				exit_flag = bool_CONTINUAR;
				goto EXIT;

			} else {

				if ( (systemVars.debugLevel &  D_GPRS ) != 0) {
					snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::prender: Modem No prendio !!\r\n\0"), u_now());
					FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
				}
			}

			// PROCESO LAS SEÑALES
			if ( pv_procesar_signals_prender( &exit_flag )) {
				// Si recibi alguna senal, debo salir.
				goto EXIT;
			}

			// No prendio: Espero 5s antes de reintentar prenderlo por SW.
			g_sleep(5);


		}

		// No prendio luego de MAX_SW_TRIES_PWRON intentos SW. Apago y prendo de nuevo
		IO_modem_hw_pwr_off();	// Apago la fuente
		g_sleep(10);			// Espero 10s antes de reintentar
	}

	// Si salgo por aqui es que el modem no prendio luego de todos los reintentos
	exit_flag = bool_RESTART;
	if ( (systemVars.debugLevel &  D_GPRS ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::prender: FAIL!! Modem No prendio en HW%d y SW%d intentos\r\n\0"), u_now(), MAX_HW_TRIES_PWRON, MAX_SW_TRIES_PWRON);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

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
static bool pv_procesar_signals_prender( bool *exit_flag )
{
	// Estoy prendiendo el modem de modo que solo me interesa
	// la senal de reload.
	// Las otras en forma implicita las estoy atendiendo

bool ret_f = false;

	if ( GPRS_stateVars.signal_reload) {
		// Salgo a reiniciar tomando los nuevos parametros.
		*exit_flag = bool_RESTART;
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
static void pv_readImei(void)
{
	// Leo el imei del modem para poder trasmitirlo al server y asi
	// llevar un control de donde esta c/sim

uint8_t i,j,start, end;


	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);

	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT+CGSN\r\0", sizeof("AT+CGSN\r\0") );
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	// Leo y Evaluo la respuesta al comando AT+CGSN
	if ( strstr( gprsRx.buffer, "OK") != NULL ) {

		g_printRxBuffer();

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
		for ( i = start; i < 64; i++ ) {
			if ( isdigit( gprs_printfBuff[i]) ) {
				buff_gprs_imei[j++] = gprs_printfBuff[i];
			} else {
				break;
			}
		}
	}

// Exit
EXIT:

	if ( (systemVars.debugLevel &  ( D_BASIC + D_GPRS ) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::prender: IMEI[%s]\r\n\0"), u_now(), buff_gprs_imei);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

}
//--------------------------------------------------------------------------------------
