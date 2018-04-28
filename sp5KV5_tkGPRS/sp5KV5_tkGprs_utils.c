/*
 * sp5KV5_tkGprs_utils.c
 *
 *  Created on: 26 de abr. de 2017
 *      Author: pablo
 */


#include "sp5KV5_tkGprs.h"

//------------------------------------------------------------------------------------
bool u_modem_prendido(void)
{
	return ( GPRS_stateVars.modem_prendido );
}
//------------------------------------------------------------------------------------
char *g_getImei(void)
{
	// Devuelve el IMEI leido al prender el modem.
	return(buff_gprs_imei);
}
//--------------------------------------------------------------------------------------
void g_sleep(uint16_t timeout)
{
	// Genero una espera contando de a 1 segundo
	while(timeout-- > 0) {
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	}

}
//------------------------------------------------------------------------------------
bool g_open_socket(void)
{
	// Envio el comando AT para abrir el socket
	// Espero hasta 5s que abra antes de salir.

uint8_t pin_dcd;
uint8_t tries;
bool exit_flag = false;

	if ( systemVars.debugLevel == D_GPRS ) {
		FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: sock opening\r\n\0" ));
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	pub_gprs_flush_RX_buffer();

	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT*E2IPO=1,\"%s\",%s\r\n\0"),systemVars.server_ip_address,systemVars.server_tcp_port);
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// Espero hasta 10s que abra
	tries = 10;
	while ( tries-- > 0 ) {
		IO_read_dcd(&pin_dcd);
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		if ( pin_dcd == 0 ) {
			exit_flag = true;
			break;
		}
	}

	return(exit_flag);
}
//------------------------------------------------------------------------------------
bool g_socket_is_open(void)
{
	// El socket esta abierto si el modem esta prendido y
	// el DCD esta en 0.
	// Cuando el modem esta apagado pin_dcd = 0
	// Cuando el modem esta prendido y el socket cerrado pin_dcd = 1
	// Cuando el modem esta prendido y el socket abierto pin_dcd = 0.

uint8_t pin_dcd;
bool exit_flag = false;

	IO_read_dcd(&pin_dcd);

	if ( ( GPRS_stateVars.modem_prendido == true ) && (pin_dcd == 0 ) ){

		if ( systemVars.debugLevel == D_GPRS ) {
			FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: sock OPEN\r\n\0" ));
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}
		exit_flag = true;

	} else {

		if ( systemVars.debugLevel == D_GPRS ) {
			FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: sock CLOSED\r\n\0") );
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}

		exit_flag = false;
	}

	return(exit_flag);

}
//------------------------------------------------------------------------------------
void g_print_debug_gprs_header(const char *msg)
{
	// Como en el gprs_printBuff tengo el frame a trasmitir, no puedo usarlo.
	// Aqui solo imprimo el header del debug.

char printfBuff[CHAR64];

	FRTOS_snprintf_P( printfBuff,sizeof(printfBuff),PSTR("%s"), msg );
	FreeRTOS_write( &pdUART1, printfBuff, sizeof(printfBuff) );

}
//------------------------------------------------------------------------------------
void pub_gprs_redial(void)
{
	GPRS_stateVars.signal_redial = true;

}
//----------------------------------------------------------------------------------------
void pub_gprs_flush_RX_buffer(void)
{

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);

	memset(gprsRx.buffer,0, UART0_RXBUFFER_LEN );
	gprsRx.ptr = 0;

}
//------------------------------------------------------------------------------------
void pub_gprs_print_RX_Buffer(void)
{

	// Imprime la respuesta a un comando.
	// Utiliza el buffer de RX.

	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: rxbuff: \r\n\0"));
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	// Imprimo todo el buffer de RX ( 640b). Sale por \0.
	FreeRTOS_write( &pdUART1, gprsRx.buffer, UART0_RXBUFFER_LEN );
	// Agrego un CRLF por las dudas
	FreeRTOS_write( &pdUART1, "\r\n\0", sizeof("\r\n\0") );

}
//------------------------------------------------------------------------------------
