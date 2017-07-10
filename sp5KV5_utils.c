/*
 * sp5KV3_utils.c
 *
 *  Created on: 27/10/2015
 *      Author: pablo
 */

#include <sp5KV5.h>

static uint8_t pv_paramLoad(uint8_t* data, uint8_t* addr, uint16_t sizebytes);
static uint8_t pv_paramStore(uint8_t* data, uint8_t* addr, uint16_t sizebytes);
static uint8_t pv_checkSum ( uint8_t *data,uint16_t sizebytes );
static void pv_convert_str_to_time_t ( char *time_str, time_t *time_struct);

//----------------------------------------------------------------------------------------
void u_panic( uint8_t panicCode )
{
char msg[16];

	snprintf_P( msg,sizeof(msg),PSTR("\r\nPANIC(%d)\r\n\0"), panicCode);
	FreeRTOS_write( &pdUART1,  msg, sizeof( msg) );
	vTaskDelay( ( TickType_t)( 20 / portTICK_RATE_MS ) );
	vTaskSuspendAll ();
	vTaskEndScheduler ();
	exit (1);
}
//----------------------------------------------------------------------------------------
bool u_configOutputs( uint8_t modo, char *param1, char *param2 )
{
	// Configura las salidas en el systemVars.
	// Manda una señal a la tkOutput.

	switch(modo) {
	case OUT_OFF:
		systemVars.outputs.modo = OUT_OFF;
		break;
	case OUT_CONSIGNA:
		systemVars.outputs.modo = OUT_CONSIGNA;
		if ( param1 != NULL ) { pv_convert_str_to_time_t(param1, &systemVars.outputs.consigna_diurna); }
		if ( param2 != NULL ) { pv_convert_str_to_time_t(param2, &systemVars.outputs.consigna_nocturna); }
		break;
	case OUT_NORMAL:
		systemVars.outputs.modo = OUT_NORMAL;
		if ( param1 != NULL ) { ( atoi(param1) == 0 )? ( systemVars.outputs.out0 = 0) : (systemVars.outputs.out0 = 1); }
		if ( param2 != NULL ) { ( atoi(param2) == 0 )? ( systemVars.outputs.out1 = 0) : (systemVars.outputs.out1 = 1); }
		break;
	}

	// tk_Output: notifico en modo persistente. Si no puedo, me voy a resetear por watchdog. !!!!
	while ( xTaskNotify(xHandle_tkOutputs, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

	return(true);

}
//----------------------------------------------------------------------------------------
bool u_configAnalogCh( uint8_t channel, char *chName, char *s_iMin, char *s_iMax, char *s_mMin, char *s_mMax )
{
	// p1 = name, p2 = iMin, p3 = iMax, p4 = mMin, p5 = mMax

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	if ( chName != NULL ) {
		memset ( systemVars.aChName[channel], '\0',   PARAMNAME_LENGTH );
		memcpy( systemVars.aChName[channel], chName , ( PARAMNAME_LENGTH - 1 ));
	}

	if ( s_iMin != NULL ) { systemVars.Imin[channel] = atoi(s_iMin); }
	if ( s_iMax != NULL ) {	systemVars.Imax[channel] = atoi(s_iMax); }
	if ( s_mMin != NULL ) {	systemVars.Mmin[channel] = atoi(s_mMin); }
	if ( s_mMax != NULL ) {	systemVars.Mmax[channel] = atof(s_mMax); }

	xSemaphoreGive( sem_SYSVars );

	return(true);

}
//----------------------------------------------------------------------------------------
bool u_configDigitalCh( uint8_t channel, char *chName, char *s_magPP )
{
	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	if ( chName != NULL ) {
		memset ( systemVars.dChName[channel], '\0',   PARAMNAME_LENGTH );
		memcpy( systemVars.dChName[channel], chName , ( PARAMNAME_LENGTH - 1 ));
	}

	if ( s_magPP != NULL ) { systemVars.magPP[channel] = atof(s_magPP); }

	xSemaphoreGive( sem_SYSVars );
	return(true);

}
//----------------------------------------------------------------------------------------
bool u_configPwrMode(uint8_t pwrMode)
{

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.pwrMode =  pwrMode;
	xSemaphoreGive( sem_SYSVars );

	// tk_aIn: notifico en modo persistente. Si no puedo, me voy a resetear por watchdog. !!!!
	while ( xTaskNotify(xHandle_tkAIn, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

	while ( xTaskNotify(xHandle_tkOutputs, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

	return(true);
}
//----------------------------------------------------------------------------------------
bool u_configTimerDial(char *s_tDial)
{
u32 tdial;

	tdial = abs( (u32) ( atol(s_tDial) ));
	if ( tdial < 120 ) { tdial = 120; }

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();
	systemVars.timerDial = tdial;
	xSemaphoreGive( sem_SYSVars );

	return(true);
}
//----------------------------------------------------------------------------------------
bool u_configTimerPoll(char *s_tPoll)
{
	// Configura el tiempo de poleo.
	// El cambio puede ser desde tkCmd o tkGprs(init frame)
	// Le avisa a la tarea tkAnalog del cambio

uint16_t tpoll;

	tpoll = abs((uint16_t) ( atol(s_tPoll) ));
	if ( tpoll < 15 ) { tpoll = 15; }

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.timerPoll = tpoll;
	xSemaphoreGive( sem_SYSVars );

	// tk_aIn: Notifico en modo persistente. Si no puedo me voy a resetear por watchdog. !!!!
	while ( xTaskNotify(xHandle_tkAIn, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

	return(true);
}
//------------------------------------------------------------------------------------
void u_configPwrSave(uint8_t modoPwrSave, char *s_startTime, char *s_endTime)
{
	// Recibe como parametros el modo ( 0,1) y punteros a string con las horas de inicio y fin del pwrsave
	// expresadas en minutos.

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.pwrSave.modo = modoPwrSave;

	if ( s_startTime != NULL ) { pv_convert_str_to_time_t( s_startTime, &systemVars.pwrSave.hora_start); }
	if ( s_endTime != NULL ) { pv_convert_str_to_time_t( s_endTime, &systemVars.pwrSave.hora_fin); }

	xSemaphoreGive( sem_SYSVars );

}
//----------------------------------------------------------------------------------------
void u_kick_Wdg( uint8_t wdgId )
{
	// Pone el correspondiente bit del wdg en 0.
	systemWdg &= ~wdgId ;

}
//------------------------------------------------------------------------------------
bool u_saveSystemParams(void)
{
	// Salva el systemVars en la EE y verifica que halla quedado bien.
	// Hago hasta 3 intentos.

bool retS = false;
uint8_t storeChecksum = 0;
uint8_t loadChecksum = 0;
uint8_t i;

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	for ( i=0; i<3; i++ ) {
		storeChecksum = pv_paramStore( (uint8_t *)&systemVars, (uint8_t *)EEADDR_SV, sizeof(systemVarsType));
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		pv_paramLoad( (uint8_t *)&tmpSV, (uint8_t *)EEADDR_SV, sizeof(systemVarsType));
		loadChecksum = pv_checkSum( (uint8_t *)&tmpSV,sizeof(systemVarsType));

		if ( loadChecksum == storeChecksum ) {
			retS = true;
			break;
		}
	}

	xSemaphoreGive( sem_SYSVars );
	return(retS);

}
//------------------------------------------------------------------------------------
bool u_loadSystemParams(void)
{
bool retS = false;
uint8_t i;

	// Leo la configuracion:  Intento leer hasta 3 veces.

	for ( i=0; i<3;i++) {
		retS =  pv_paramLoad( (uint8_t *)&systemVars, (uint8_t *)EEADDR_SV, sizeof(systemVarsType));
		if ( retS )
			break;
	}

	// Ajustes de inicio:
	strncpy_P(systemVars.dlg_ip_address, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;
	systemVars.ri = 0;
	//systemVars.debugLevel = D_BASIC;
	systemVars.wrkMode = WK_NORMAL;
	systemVars.terminal_on = false;

	// Cuando arranca si la EE no esta inicializada puede dar cualquier cosa.
	// De este modo controlo el largo de los strings.
	systemVars.dlgId[DLGID_LENGTH - 1] = '\0';
	systemVars.apn[APN_LENGTH - 1] = '\0';
	systemVars.server_tcp_port[PORT_LENGTH - 1] = '\0';
	systemVars.server_ip_address[IP_LENGTH - 1] = '\0';
	systemVars.dlg_ip_address[IP_LENGTH - 1] = '\0';
	systemVars.serverScript[SCRIPT_LENGTH - 1] = '\0';
	systemVars.passwd[PASSWD_LENGTH - 1] = '\0';

	// Nombre de los canales
	systemVars.aChName[0][PARAMNAME_LENGTH - 1] = '\0';
	systemVars.aChName[1][PARAMNAME_LENGTH - 1] = '\0';
	systemVars.aChName[2][PARAMNAME_LENGTH - 1] = '\0';

	systemVars.dChName[0][PARAMNAME_LENGTH - 1] = '\0';
	systemVars.dChName[1][PARAMNAME_LENGTH - 1] = '\0';

	return(retS);

}
//------------------------------------------------------------------------------------
void u_loadDefaults(void)
{
uint8_t channel;

// Configura el systemVars con valores por defecto.

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.initByte = 0x49;
	strncpy_P(systemVars.dlgId, PSTR("DEF400\0"),DLGID_LENGTH);
	strncpy_P(systemVars.server_tcp_port, PSTR("80\0"),PORT_LENGTH	);
	strncpy_P(systemVars.passwd, PSTR("spymovil123\0"),PASSWD_LENGTH);
	strncpy_P(systemVars.serverScript, PSTR("/cgi-bin/sp5K/sp5K.pl\0"),SCRIPT_LENGTH);

	systemVars.csq = 0;
	systemVars.dbm = 0;
	systemVars.gsmBand = 8;
	systemVars.ri = 0;
	systemVars.wrkMode = WK_NORMAL;
	systemVars.pwrMode = PWR_DISCRETO;
	systemVars.terminal_on = false;

	strncpy_P(systemVars.apn, PSTR("SPYMOVIL.VPNANTEL\0"),APN_LENGTH);
	systemVars.roaming = false;

	// DEBUG
	systemVars.debugLevel = D_BASIC;

	strncpy_P(systemVars.server_ip_address, PSTR("192.168.0.9\0"),IP_LENGTH);
	systemVars.timerPoll = 300;			// Poleo c/5 minutos
	systemVars.timerDial = 1800;		// Transmito c/3 hs.

	// Todos los canales quedan por default en 0-20mA, 0-6k.
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS ; channel++) {
		systemVars.Imin[channel] = 0;
		systemVars.Imax[channel] = 20;
		systemVars.Mmin[channel] = 0;
		systemVars.Mmax[channel] = 6.0;
	}

	strncpy_P(systemVars.aChName[0], PSTR("pA\0"),3);
	strncpy_P(systemVars.aChName[1], PSTR("pB\0"),3);
	strncpy_P(systemVars.aChName[2], PSTR("pC\0"),3);

	// Canales digitales
	strncpy_P(systemVars.dChName[0], PSTR("v0\0"),3);
	systemVars.magPP[0] = 0.1;
	strncpy_P(systemVars.dChName[1], PSTR("v1\0"),3);
	systemVars.magPP[1] = 0.1;

	// Detector de Tilt.
	systemVars.tiltEnabled = false;

	// Salidas:
	systemVars.outputs.modo = OUT_OFF;
	systemVars.outputs.out0 = 0;
	systemVars.outputs.out1 = 0;
	systemVars.outputs.consigna_diurna.hour = 05;
	systemVars.outputs.consigna_diurna.min = 30;
	systemVars.outputs.consigna_nocturna.hour = 23;
	systemVars.outputs.consigna_nocturna.min = 30;

	// PwrSave
	systemVars.pwrSave.modo = modoPWRSAVE_ON;
	systemVars.pwrSave.hora_start.hour = 23;
	systemVars.pwrSave.hora_start.min = 30;
	systemVars.pwrSave.hora_fin.hour = 5;
	systemVars.pwrSave.hora_fin.min = 30;

	xSemaphoreGive( sem_SYSVars );


}
//------------------------------------------------------------------------------------
char *u_now(void)
{

	// Devuelve un puntero a un string con la fecha y hora formateadas para usar en
	// los mensajes de log.

RtcTimeType_t rtcDateTime;

	RTC_read(&rtcDateTime);
	rtcDateTime.year -= 2000;
	snprintf_P( nowStr,sizeof(nowStr), PSTR("%02d/%02d/%02d %02d:%02d:%02d\0"),rtcDateTime.day,rtcDateTime.month,rtcDateTime.year,rtcDateTime.hour,rtcDateTime.min,rtcDateTime.sec );
	return(nowStr);
}
//------------------------------------------------------------------------------------
void u_debugPrint(uint8_t debugCode, char *msg, uint16_t size)
{

	if ( (systemVars.debugLevel & debugCode) != 0) {
		FreeRTOS_write( &pdUART1, msg, size );
	}
}
//------------------------------------------------------------------------------------
void u_reset(void)
{
	wdt_enable(WDTO_30MS);
	while(1) {}
}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static uint8_t pv_paramLoad(uint8_t* data, uint8_t* addr, uint16_t sizebytes)
{
uint16_t i;
uint8_t checksum_stored=0;
uint8_t checksum=0;

	// load parameters
	eeprom_read_block(data, (uint8_t *)addr, sizebytes);
	// load checksum
	eeprom_read_block(&checksum_stored, (uint8_t *)(addr+sizebytes), sizeof(uint8_t));

	// calculate own checksum
	for(i=0;i<sizebytes;i++)
		checksum += data[i];
	checksum = ~checksum;

	if(checksum == checksum_stored)
		return true;
	else
		return false;
}
//------------------------------------------------------------------------------------
static uint8_t pv_paramStore(uint8_t* data, uint8_t* addr, uint16_t sizebytes)
{
	// Almacena un string de bytes en la eeprom interna del micro

uint16_t i;
uint8_t checksum=0;

	// calculate checksum
	for(i=0;i<sizebytes;i++)
		checksum += data[i];
	checksum = ~checksum;

	// store parameters
	 eeprom_write_block(data, (uint8_t *)addr, sizebytes);
	// store checksum
	eeprom_write_block(&checksum, (uint8_t *)(addr+sizebytes), sizeof(uint8_t));

	return(checksum);
}
//------------------------------------------------------------------------------------
static uint8_t pv_checkSum ( uint8_t *data,uint16_t sizebytes )
{
uint16_t i;
uint8_t checksum=0;

	// calculate checksum
	for(i=0;i<sizebytes;i++)
		checksum += data[i];
	checksum = ~checksum;
	return(checksum);
}
//------------------------------------------------------------------------------------
static void pv_convert_str_to_time_t ( char *time_str, time_t *time_struct )
{

	// Convierte un string hhmm en una estructura time_type que tiene
	// un campo hora y otro minuto

uint16_t time_num;

	time_num = atol(time_str);
	time_struct->hour = (uint8_t) (time_num / 100);
	time_struct->min = (uint8_t)(time_num % 100);

}
//------------------------------------------------------------------------------------
