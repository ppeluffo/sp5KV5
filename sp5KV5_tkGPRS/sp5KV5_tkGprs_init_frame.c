/*
 * sp5KV5_tkGprs_init_frame.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV5_tkGprs.h"

bool pv_send_init_frame(void);
bool pv_process_init_response(void);
static void pv_TX_init_frame(void);
static void pv_reconfigure_params(void);

static void pv_process_server_clock(void);
static uint8_t pv_process_pwrSave(void);
static uint8_t pv_process_timerPoll(void);
static uint8_t pv_process_timerDial(void);
static uint8_t pv_process_digitalCh(uint8_t channel);
static uint8_t pv_process_AnalogCh(uint8_t channel);
static uint8_t pv_process_tilt(void);
static uint8_t pv_process_Outputs(void);

//------------------------------------------------------------------------------------
bool gprs_init_frame(void)
{
	// Debo mandar el frame de init al server, esperar la respuesta, analizarla
	// y reconfigurarme.
	// Intento 3 veces antes de darme por vencido.
	// El socket puede estar abierto o cerrado. Lo debo determinar en c/caso y
	// si esta cerrado abrirlo.
	// Mientras espero la respuesta debo monitorear que el socket no se cierre

uint8_t intentos;
bool exit_flag = false;

// Entry:

	GPRS_stateVars.state = G_INIT_FRAME;

	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe:\r\n\0"), u_now());
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// Intenteo MAX_INIT_TRYES procesar correctamente el INIT
	for ( intentos = 0; intentos < MAX_INIT_TRYES; intentos++ ) {

		if ( ! pv_send_init_frame() ) {			// Intento madar el frame al servidor
			if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
				snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Send retry\r\n\0"), u_now() );
				FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
			}
			continue;
		}

		if ( ! pv_process_init_response() ) {	// Intento procesar la respuesta
			if ( ( systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
				snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Rsp retry\r\n\0"), u_now() );
				FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
			}
			continue;
		}

		// Aqui es que anduvo todo bien y debo salir para pasar al modo DATA
		if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
			snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Init frame OK.\r\n\0"), u_now() );
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}

		exit_flag = true;
		goto EXIT;

	}

	// Aqui es que no puede enviar/procesar el INIT correctamente
	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Init frame FAIL !!.\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

// Exit
EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
bool pv_send_init_frame(void)
{
	// Intento enviar 1 SOLO frame de init.
	// El socket puede estar cerrado por lo que reintento abrirlo hasta 3 veces.
	// Una vez que envie el INIT, salgo.
	// Al entrar, veo que el socket este cerrado.

uint8_t i;
bool exit_flag = false;

	for ( i = 0; i < MAX_TRYES_OPEN_SOCKET; i++ ) {

		//g_sleep(5);			// Espero 5s
		if ( g_socket_is_open() ) {
			pv_TX_init_frame();		// Escribo en el socket el frame de INIT
			exit_flag = true;
			break;
		}

		g_open_socket();
	}

	return(exit_flag);
}
//------------------------------------------------------------------------------------
bool pv_process_init_response(void)
{
	// Espero la respuesta al frame de INIT.
	// Si la recibo la proceso.
	// Salgo por timeout 10s o por socket closed.

uint8_t timeout;
bool exit_flag = false;

	for ( timeout = 0; timeout < 10; timeout++) {

		g_sleep(1);				// Espero 1s

		if ( ! g_socket_is_open() ) {		// El socket se cerro
			exit_flag = false;
			goto EXIT;
		}

		if ( strstr( gprsRx.buffer, "ERROR") != NULL ) {	// Recibi un ERROR de respuesta
			g_printRxBuffer();
			exit_flag = false;
			goto EXIT;
		}

		if ( strstr( gprsRx.buffer, "INIT_OK") != NULL ) {	// Respuesta correcta del server
			g_printRxBuffer();
			pv_reconfigure_params();
			exit_flag = true;
			goto EXIT;
		}

	}

// Exit:
EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
static void pv_TX_init_frame(void)
{
	// Send Init Frame
	// GET /cgi-bin/sp5K/sp5K.pl?DLGID=SPY001&PASSWD=spymovil123&&INIT&ALARM&PWRM=CONT&TPOLL=23&TDIAL=234&PWRS=1,1230,2045&A0=pZ,1,20,3,10&D0=qE,3.24&CONS=1,1234,927,1,3 HTTP/1.1
	// Host: www.spymovil.com
	// Connection: close\r\r ( no mando el close )

uint16_t pos = 0;
uint8_t i;

	if ( (systemVars.debugLevel &  D_GPRS ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Sent\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// Trasmision: 1r.Parte.
	// HEADER:
	// Envio parcial ( no CR )
	memset( gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
	g_flushRXBuffer();

	pos = snprintf_P( gprs_printfBuff,CHAR256,PSTR("GET " ));
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("%s"), systemVars.serverScript );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("?DLGID=%s"), systemVars.dlgId );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&PASSWD=%s"), systemVars.passwd );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&IMEI=%s"), g_getImei() );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&VER=%s\0"), SP5K_REV );
	// GPRS sent
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// DEBUG & LOG
	if ( (systemVars.debugLevel &  D_GPRS ) != 0) {
		snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("\r\n\0") );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// BODY ( 1a parte) :
	memset(gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	if ( u_tilt_alarmFired() ) {
		pos = snprintf_P( gprs_printfBuff ,CHAR256,PSTR("&INIT&ALARM"));
	} else {
		pos = snprintf_P( gprs_printfBuff ,CHAR256,PSTR("&INIT"));
	}

	// timerpoll
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&TPOLL=%d"), systemVars.timerPoll);

	// timerdial
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&TDIAL=%d"), systemVars.timerDial);

	// tilt
	if ( systemVars.tiltEnabled == TRUE) {
		pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&TILT=ON"));
	} else {
		pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&TILT=OFF"));
	}

	// pwrSave
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&PWRS=%d,"),systemVars.pwrSave.modo);
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("%02d%02d,"),systemVars.pwrSave.hora_start.hour, systemVars.pwrSave.hora_start.min );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("%02d%02d"), systemVars.pwrSave.hora_fin.hour, systemVars.pwrSave.hora_fin.min);

	// outputs:
	if ( systemVars.outputs.modo == OUT_OFF ) {
		pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&OUTS=0,0,0"));
	} else if ( systemVars.outputs.modo == OUT_CONSIGNA ) {
		pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&OUTS=1,%02d%02d,%02d%02d"),systemVars.outputs.consigna_diurna.hour,systemVars.outputs.consigna_diurna.min,systemVars.outputs.consigna_nocturna.hour,systemVars.outputs.consigna_nocturna.min);
	} else if ( systemVars.outputs.modo == OUT_NORMAL ) {
		// En modo normal no importan los valores de out0,out1 porque son dinamicos.
		pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&OUTS=2,0,0"));
	}

	// csq
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&CSQ=%d\0"), systemVars.csq);

	// GPRS sent
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// DEBUG & LOG
	if ( (systemVars.debugLevel &  D_GPRS ) != 0) {
		snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("\r\n\0") );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// BODY ( 2a parte) :
	memset(gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	pos = 0;

	// Configuracion de canales analogicos
	for ( i = 0; i < NRO_ANALOG_CHANNELS; i++) {
		pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&A%d=%s,%d,%d,%d,%.02f"), i,systemVars.aChName[i],systemVars.Imin[i], systemVars.Imax[i], systemVars.Mmin[i], systemVars.Mmax[i]);
	}
	// Configuracion de canales digitales
	for ( i = 0; i < NRO_DIGITAL_CHANNELS; i++) {
		pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&D%d=%s,%.02f"),i,systemVars.dChName[i],systemVars.magPP[i]);
	}

	// Reset status
	pos += snprintf_P( &gprs_printfBuff[pos],( CHAR256 - pos ),PSTR("&WDG=%d\0"),wdgStatus.resetCause );

	// GPRS send
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// DEBUG & LOG
	if ( (systemVars.debugLevel &  D_GPRS ) != 0) {
		snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("\r\n\0") );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// TAIL ( No mando el close ya que espero la respuesta y no quiero que el socket se cierre )
	memset(gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	pos = snprintf_P( gprs_printfBuff, ( sizeof(gprs_printfBuff) - pos ),PSTR(" HTTP/1.1\n") );
	pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ),PSTR("Host: www.spymovil.com\n" ));
	pos += snprintf_P( &gprs_printfBuff[pos], sizeof(gprs_printfBuff),PSTR("\n\n\0" ));

	// GPRS sent
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// DEBUG & LOG
	if ( (systemVars.debugLevel &  D_GPRS ) != 0) {
		snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("\r\n\0") );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
static void pv_reconfigure_params(void)
{

uint8_t saveFlag = 0;

	// Proceso la respuesta del INIT para reconfigurar los parametros
	pv_process_server_clock();
	saveFlag += pv_process_timerPoll();
	saveFlag += pv_process_timerDial();
	saveFlag += pv_process_pwrSave();
	saveFlag += pv_process_tilt();
	// Canales analogicos.
	saveFlag += pv_process_AnalogCh(0);
	saveFlag += pv_process_AnalogCh(1);
	saveFlag += pv_process_AnalogCh(2);
	// Canales digitales
	saveFlag += pv_process_digitalCh(0);
	saveFlag += pv_process_digitalCh(1);

	// Consignas
	saveFlag += pv_process_Outputs();

	if ( saveFlag > 0 ) {

		if ( u_saveSystemParams() ) {

			// DEBUG & LOG
			if ( (systemVars.debugLevel &  D_GPRS ) != 0) {
				snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Save params OK\r\n\0"), u_now());
				FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
			}

		}
	}

}
//------------------------------------------------------------------------------------
static void pv_process_server_clock(void)
{
/* Extraigo el srv clock del string mandado por el server y si el drift con la hora loca
 * es mayor a 5 minutos, ajusto la hora local
 * La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:PWRM=DISC:</h1>
 *
 */

char *p, *s;
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char rtcStr[12];

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "CLOCK");
	if ( p == NULL ) {
		return;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);			// CLOCK

	token = strsep(&stringp,delim);			// rtc
	memset(rtcStr, '\0', sizeof(rtcStr));
	memcpy(rtcStr,token, sizeof(rtcStr));	// token apunta al comienzo del string con la hora
	RTC_str_to_date(rtcStr);

	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Update rtc to: %s\r\n\0"), u_now(), rtcStr );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
static uint8_t pv_process_timerPoll(void)
{
//	La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:TPOLL=600:PWRM=DISC:</h1>

char *p, *s;
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "TPOLL");
	if ( p == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	// TPOLL

	token = strsep(&stringp,delim);	// timerPoll
	u_configTimerPoll(token);
	ret = 1;
	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Reconfig TPOLL\r\n\0"), u_now());
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

quit:

	return(ret);
}
//------------------------------------------------------------------------------------
static uint8_t pv_process_timerDial(void)
{
	//	La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:CD=1230:CN=0530</h1>

char *p, *s;
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "TDIAL");
	if ( p == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	// TDIAL

	token = strsep(&stringp,delim);	// timerDial
	u_configTimerDial(token);
	ret = 1;
	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Reconfig TDIAL\r\n\0"), u_now());
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

quit:

	return(ret);
}
//------------------------------------------------------------------------------------
static uint8_t pv_process_pwrSave(void)
{
//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRS=1,2230,0600:D0=q0,1.00:D1=q1,1.00</h1>
//  Las horas estan en formato HHMM.

u08 ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *p1,*p2;
u08 modo;
char *p, *s;

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "PWRS");
	if ( p == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	//PWRS

	token = strsep(&stringp,delim);	// modo
	modo = atoi(token);
	p1 = strsep(&stringp,delim);	// startTime
	p2 = strsep(&stringp,delim); 	// endTime

	u_configPwrSave(modo, p1, p2);
	ret = 1;
	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Reconfig PWRSAVE\r\n\0"), u_now());
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

quit:
	return(ret);

}
//--------------------------------------------------------------------------------------
static uint8_t pv_process_tilt(void)
{
//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:TILT=ON:PWRS=1,2230,0600:D0=q0,1.00:D1=q1,1.00</h1>

uint8_t ret = 0;
char *p, *s;

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);

	p = strstr(s, "TILT=ON");
	if ( p != NULL ) {
		ret = 1;
		systemVars.tiltEnabled = TRUE;
		if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
			snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRSE::iniframe: Reconfig TILT ON\r\n\0"), u_now());
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}
		goto quit;
	}

	p = strstr(s, "TILT=OFF");
	if ( p != NULL ) {
		ret = 1;
		systemVars.tiltEnabled = FALSE;
		if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
			snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Reconfig TILT OFF\r\n\0"), u_now());
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}
		goto quit;
	}

quit:

	return(ret);

}
//--------------------------------------------------------------------------------------
static uint8_t pv_process_AnalogCh(uint8_t channel)
{
//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:A0=pA,0,20,0,6:A1=pB,0,20,0,10:A2=pC,0,20,0,10:D0=q0,1.00:D1=q1,1.00</h1>

uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *chName,*s_iMin,*s_iMax,*s_mMin,*s_mMax;
char *s;

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);

	switch (channel) {
	case 0:
		stringp = strstr(s, "A0=");
		break;
	case 1:
		stringp = strstr(s, "A1=");
		break;
	case 2:
		stringp = strstr(s, "A2=");
		break;
	default:
		ret = 0;
		goto quit;
		break;
	}

	if ( stringp == NULL ) {
		ret = 0;
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,stringp,31);

	stringp = localStr;
	token = strsep(&stringp,delim);	//A0

	chName = strsep(&stringp,delim);	//name
	s_iMin = strsep(&stringp,delim);	//iMin
	s_iMax = strsep(&stringp,delim);	//iMax
	s_mMin = strsep(&stringp,delim);	//mMin
	s_mMax = strsep(&stringp,delim);	//mMax

	u_configAnalogCh( channel, chName,s_iMin,s_iMax,s_mMin,s_mMax );
	ret = 1;
	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Reconfig A%d\r\n\0"), u_now(), channel);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

quit:
	return(ret);
}
//--------------------------------------------------------------------------------------
static uint8_t pv_process_digitalCh(uint8_t channel)
{

//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:A0=pA,0,20,0,6:A1=pB,0,20,0,10:A2=pC,0,20,0,10:D0=q0,1.00:D1=q1,1.00</h1>

uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *chName, *s_magPP;
char *s;

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	switch (channel) {
	case 0:
		stringp = strstr(s, "D0=");
		break;
	case 1:
		stringp = strstr(s, "D1=");
		break;
	default:
		ret = 0;
		goto quit;
		break;
	}

	if ( stringp == NULL ) {
		ret = 0;
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,stringp,31);

	stringp = localStr;
	token = strsep(&stringp,delim);	//D0

	chName = strsep(&stringp,delim);	//name
	s_magPP = strsep(&stringp,delim);	//magPp
	u_configDigitalCh( channel, chName, s_magPP );
	ret = 1;
	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Reconfig D%d\r\n\0"), u_now(), channel);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

quit:

	return(ret);

}
//--------------------------------------------------------------------------------------
static uint8_t pv_process_Outputs(void)
{
	//	La linea recibida es del tipo:
	//	<h1>INIT_OK:OUTS=modo,param1,param2:</h1>
	// modo=0: OUTS_OFF
	// modo=1: OUTS_NORMAL. param1,param2 indican los valores de las salidas
	// modo=2: OUTS_CONSIGNAS: param1, param2 son las horas de la consigna diurna y la nocturna
	//  Las horas estan en formato HHMM.

uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *p1,*p2;
uint8_t modo;
char *p, *s;

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "OUTS");
	if ( p == NULL )
		goto quit;

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	//OUTS

	token = strsep(&stringp,delim);	// modo
	modo = atoi(token);
	p1 = strsep(&stringp,delim);	// startTime
	p2 = strsep(&stringp,delim); 	// endTime

	u_configOutputs(modo, p1, p2);
	ret = 1;

	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Reconfig OUTPUTS\r\n\0"), u_now());
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

quit:
	return(ret);

}
//--------------------------------------------------------------------------------------
