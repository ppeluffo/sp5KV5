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

//------------------------------------------------------------------------------------
void pub_convert_str_to_time_t ( char *time_str, time_t *time_struct )
{

	// Convierte un string hhmm en una estructura time_type que tiene
	// un campo hora y otro minuto

uint16_t time_num;

	time_num = atol(time_str);
	time_struct->hour = (uint8_t) (time_num / 100);
	time_struct->min = (uint8_t)(time_num % 100);

}
//----------------------------------------------------------------------------------------
void pub_uarts_ctl(uint8_t cmd)
{

static bool terminal_prendida = false;
static bool modem_prendido = false;

	switch(cmd) {

	case MODEM_PRENDER:
		// Si tengo el LM365 deshabilitado lo habilito
		if ( ! modem_prendido ) {
			cbi(UARTCTL_PORT, UARTCTL);	// Habilito el LM365
			modem_prendido = true;
		}
		break;

	case TERM_PRENDER:
		if ( ! terminal_prendida ) {
			cbi(UARTCTL_PORT, UARTCTL);	// Habilito el LM365
			terminal_prendida = true;
		}
		break;

	case MODEM_APAGAR:
		modem_prendido = false;
		if ( ! terminal_prendida ) {
			sbi(UARTCTL_PORT, UARTCTL);	// Deshabilito el LM365
		}
		break;

	case TERM_APAGAR:
		terminal_prendida = false;
		if ( ! modem_prendido ) {
			sbi(UARTCTL_PORT, UARTCTL);	// Deshabilito el LM365
		}
		break;
	}

}
//----------------------------------------------------------------------------------------
bool pub_configTimerDial(char *s_tDial)
{
uint32_t tdial;

	// tdial es el tiempo de discado en segundos.
	// Puede ser 0 ( CONTINUO ) o mayor a 900 ( 15 minutos )

	tdial = (uint32_t) ( atol(s_tDial) );
	if ( tdial < 900 ) {
		tdial = 0;
	}

	if ( tdial > 86400 ) {
		tdial = 86400;
	}

	systemVars.timerDial = tdial;

	return(true);
}
//----------------------------------------------------------------------------------------
#ifdef SP5KV5_3CH
void pub_configPwrSave(uint8_t modoPwrSave, char *s_startTime, char *s_endTime)
{
	// Recibe como parametros el modo ( 0,1) y punteros a string con las horas de inicio y fin del pwrsave
	// expresadas en minutos.

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.pwrSave.modo = modoPwrSave;

	if ( s_startTime != NULL ) { pub_convert_str_to_time_t( s_startTime, &systemVars.pwrSave.hora_start); }
	if ( s_endTime != NULL ) { pub_convert_str_to_time_t( s_endTime, &systemVars.pwrSave.hora_fin); }

	xSemaphoreGive( sem_SYSVars );

}
#endif /* SP5KV5_3CH */
//----------------------------------------------------------------------------------------
bool pub_saveSystemParams(void)
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
bool pub_loadSystemParams(void)
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

	// Siempre al arrancar el debug es none
	systemVars.debugLevel = D_NONE;

	return(retS);

}
//------------------------------------------------------------------------------------
void pub_loadDefaults(void)
{

// Configura el systemVars con valores por defecto.

	systemVars.initByte = 0x49;

#ifdef SERVER_SPY
	strncpy_P(systemVars.dlgId, PSTR("DEF400\0"),DLGID_LENGTH);
	strncpy_P(systemVars.server_tcp_port, PSTR("80\0"),PORT_LENGTH	);
	strncpy_P(systemVars.passwd, PSTR("spymovil123\0"),PASSWD_LENGTH);
	strncpy_P(systemVars.apn, PSTR("SPYMOVIL.VPNANTEL\0"),APN_LENGTH);
	strncpy_P(systemVars.serverScript, PSTR("/cgi-bin/sp5K/sp5K.pl\0"),SCRIPT_LENGTH);
	strncpy_P(systemVars.server_ip_address, PSTR("192.168.0.9\0"),IP_LENGTH);

#endif

#ifdef SERVER_OSE
	strncpy_P(systemVars.dlgId, PSTR("DEF400\0"),DLGID_LENGTH);
	strncpy_P(systemVars.server_tcp_port, PSTR("80\0"),PORT_LENGTH	);
	strncpy_P(systemVars.passwd, PSTR("spymovil123\0"),PASSWD_LENGTH);
	strncpy_P(systemVars.apn, PSTR("STG1.VPNANTEL\0"),APN_LENGTH);
	strncpy_P(systemVars.serverScript, PSTR("/cgi-bin/sp5K/sp5K.pl\0"),SCRIPT_LENGTH);
	strncpy_P(systemVars.server_ip_address, PSTR("172.27.0.26\0"),IP_LENGTH);
#endif

#ifdef SERVER_UTE
	strncpy_P(systemVars.dlgId, PSTR("DEF400\0"),DLGID_LENGTH);
	strncpy_P(systemVars.server_tcp_port, PSTR("80\0"),PORT_LENGTH	);
	strncpy_P(systemVars.passwd, PSTR("spymovil123\0"),PASSWD_LENGTH);
	strncpy_P(systemVars.apn, PSTR("SPYMOVIL.VPNANTEL\0"),APN_LENGTH);
	strncpy_P(systemVars.serverScript, PSTR("/cgi-bin/sp5K/sp5K8CH.pl\0"),SCRIPT_LENGTH);
	strncpy_P(systemVars.server_ip_address, PSTR("192.168.1.9\0"),IP_LENGTH);
#endif

	systemVars.csq = 0;
	systemVars.dbm = 0;
	systemVars.gsmBand = 8;
	systemVars.ri = 0;
	systemVars.terminal_on = false;
	systemVars.roaming = false;

	// DEBUG
	systemVars.debugLevel = D_NONE;

	// Analogico
	pub_analog_load_defaults();

	// digital
	pub_digital_load_defaults();

#ifdef SP5KV5_3CH

	systemVars.timerDial = 1800;		// Transmito c/3 hs.

	// Salidas:
	pub_outputs_load_defaults();

	// PwrSave
	systemVars.pwrSave.modo = modoPWRSAVE_ON;
	systemVars.pwrSave.hora_start.hour = 23;
	systemVars.pwrSave.hora_start.min = 30;
	systemVars.pwrSave.hora_fin.hour = 5;
	systemVars.pwrSave.hora_fin.min = 30;
#endif /* SP5KV5_3CH */

#ifdef SP5KV5_8CH

	systemVars.timerDial = 0;		// Continuo

#endif /* SP5KV5_8CH */
}
//------------------------------------------------------------------------------------
void pub_reset(void)
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

	taskENTER_CRITICAL();

	// load parameters
	eeprom_read_block(data, (uint8_t *)addr, sizebytes);
	// load checksum
	eeprom_read_block(&checksum_stored, (uint8_t *)(addr+sizebytes), sizeof(uint8_t));

	taskEXIT_CRITICAL();

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

	taskENTER_CRITICAL();

	// store parameters
	eeprom_write_block(data, (uint8_t *)addr, sizebytes);
	// store checksum
	eeprom_write_block(&checksum, (uint8_t *)(addr+sizebytes), sizeof(uint8_t));

	taskEXIT_CRITICAL();

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
void debug_test_printf(void)
{
	char *ptr = "Hello world!";
	char *np = 0;
	int i = 5;
	unsigned int bs = sizeof(int)*8;
	int mi;
	char buf[80];

	mi = (1 << (bs-1)) + 1;

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("%s\r\n"), ptr);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("printf test\r\n"));
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("%s is null pointer\r\n"), np);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("%d = 5\r\n"), i);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("%d = - max int\r\n"), mi);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("char %c = 'a'\r\n"), 'a');
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("hex %x = ff\r\n"), 0xff);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("hex %02x = 00\r\n"), 0);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("signed %d = unsigned %u = hex %x\r\n"), -3, -3, -3);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("%d %s(s)%\r\n"), 0, "message");
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR(" 3: %04d zero padded\r\n"), 3);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR(" 4: %-4d left justif.\n"), 3);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR(" 5: %4d right justif.\r\n"), 3);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("-6: %04d zero padded\r\n"), -3);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("-7: %-4d left justif.\r\n"), -3);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("-8: %4d right justif.\r\n"), -3);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

}
//------------------------------------------------------------------------------------
void debug_print_self_stack_watermark(uint8_t trace)
{

UBaseType_t uxHighWaterMark;

	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("GTX: wtm(%d)=%d\r\n\0"),trace,uxHighWaterMark);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

}
//------------------------------------------------------------------------------------

