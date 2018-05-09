/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */

#include <sp5KV5.h>
#include "sp5KV5_tkGPRS/sp5KV5_tkGprs.h"

static char cmd_printfBuff[CHAR128];
char *argv[16];

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);
static uint8_t pv_makeArgv(void);

#ifdef SP5KV5_3CH
	static void pv_cmd_wrOUT8814(void);
#endif /* SP5KV5_3CH */

bool pv_cmdWrDebugLevel(char *s);
static void pv_readMemory(void);

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdClearScreen(void);
static void cmdHelpFunction(void);
static void cmdResetFunction(void);
static void cmdStatusFunction(void);
static void cmdReadFunction(void);
static void cmdWriteFunction(void);
static void cmdKillFunction(void);
static void cmdConfigFunction(void);
static void pv_cmd_rwGPRS(uint8_t cmd_mode );

#define WR_CMD 0
#define RD_CMD 1

// La tarea pasa por el mismo lugar c/250ms.
// Cuando hago read frame, espera 10s
// Cuando hago reset memory espera 30s
// Cuando hago read memory idem.
#define WDG_CMD_TIMEOUT	60
/*------------------------------------------------------------------------------------*/
void tkCmd(void * pvParameters)
{

uint8_t c;
uint8_t ticks;
( void ) pvParameters;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	cmdlineInit();
	cmdlineSetOutputFunc(pvFreeRTOS_UART1_writeChar);

	cmdlineAddCommand((uint8_t *)("cls"), cmdClearScreen );
	cmdlineAddCommand((uint8_t *)("help"), cmdHelpFunction);
	cmdlineAddCommand((uint8_t *)("reset"), cmdResetFunction);
	cmdlineAddCommand((uint8_t *)("read"), cmdReadFunction);
	cmdlineAddCommand((uint8_t *)("write"), cmdWriteFunction);
	cmdlineAddCommand((uint8_t *)("status"), cmdStatusFunction);
	cmdlineAddCommand((uint8_t *)("kill"), cmdKillFunction);
	cmdlineAddCommand((uint8_t *)("config"), cmdConfigFunction);

	// Espero la notificacion para arrancar
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("starting tkCmd..\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	ticks = 1;
	FreeRTOS_ioctl( &pdUART1,ioctlSET_TIMEOUT, &ticks, false );

	// loop
	for( ;; )
	{
		pub_control_watchdog_kick(WDG_CMD, WDG_CMD_TIMEOUT);

		if ( pub_control_terminal_is_on() ) {
			// Solo si la terminal esta prendida leo datos

			c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
			// el read se bloquea 50ms. lo que genera la espera.
			while ( FreeRTOS_read( &pdUART1, &c, 1 ) == 1 ) {
				cmdlineInputFunc(c);
			}

			/* run the cmdline execution functions */
			cmdlineMainLoop();

		} else {
			// Genero una espera para poder entrar en sleep mode
			vTaskDelay( ( TickType_t)( 250 / portTICK_RATE_MS ) );
		}

	}
}
/*------------------------------------------------------------------------------------*/
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\x1B[2J\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void cmdHelpFunction(void)
{

	pv_makeArgv();

	// HELP WRITE
	if (!strcmp_P( strupr(argv[1]),PSTR("WRITE\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-write\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#ifdef SP5KV5_3CH
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  consigna {diurna|nocturna}, outputs {x,x}\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  out sleep|reset|phase(A/B)|enable(A/B) {0|1}\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  out pulse (A/B) (+/-) (ms)\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  termpwr {0|1},sensorpwr {0|1},analogpwr {0|1}\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

#endif /* SP5KV5_3CH */

#ifdef SP5KV5_8CH
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ina {0,1,2} conf {value}\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#endif /* SP5KV5_8CH */

		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  mcp {devId}{regAddr}{regValue}\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  clearQ {0|1}\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee addr string\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  gprs cmd {atcmd}, redial, (pwr|sw) {on|off}\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	// HELP READ
	else if (!strcmp_P( strupr(argv[1]), PSTR("READ\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR( "-read\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR( "  mcp id regAddr\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

#ifdef SP5KV5_8CH
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ina {0,1,2} {conf|chXshv|chXbusv|mfid|dieid}\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#endif /* SP5KV5_8CH */

		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  an chId\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  din chId\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee {addr} {lenght}\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc, frame, memory\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  gprs (rsp,dcd)\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	// HELP RESET
	else if (!strcmp_P( strupr(argv[1]), PSTR("RESET\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("-reset\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  memory\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	// HELP CONFIG
	else if (!strcmp_P( strupr(argv[1]), PSTR("CONFIG\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("-config\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  rtc YYMMDDhhmm\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  timerpoll, dlgid, gsmband\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  debug {none,mem,gprs,analog,digital,outputs} \r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  analog chId aname imin imax mmin mmax\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#ifdef SP5KV5_8CH
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  cspan {0..8} {value}\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#endif /* SP5KV5_8CH */
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  digital chId dname magp\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  apn, roaming {on|off}, port, ip, script, passwd\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#ifdef SP5KV5_3CH
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  timerdial, terminal {on|off} \r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  pwrsave modo [{on|off}] [{hhmm1}, {hhmm2}]\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  outputs {off}|{normal o0 o1}|{consigna hhmm_dia hhmm_noche}\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#endif /* SP5KV5_3CH */
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  defaults \r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  save\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	// HELP KILL
	else if (!strcmp_P( strupr(argv[1]), PSTR("KILL\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("-kill \r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
#ifdef SP5KV5_3CH
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  {analog,digital,outputs,gprstx,gprsrx}\r\n\0"));
#else
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  {analog,digital,gprstx,gprsrx}\r\n\0"));
#endif /* SP5KV5_3CH */
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	} else {

		// HELP GENERAL
		memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SP5K_MODELO, SP5K_VERSION, SP5K_REV, SP5K_DATE);
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("Available commands are:\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("-cls\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("-status\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("-reset...\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("-config...\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("-read...\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("-write...\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("-kill...\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

}
/*------------------------------------------------------------------------------------*/
static void cmdResetFunction(void)
{
	pv_makeArgv();

	// Reset memory ??
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))) {
		FF_rewind();
	}

	cmdClearScreen();
	// RESET
	pub_reset();

}
/*------------------------------------------------------------------------------------*/
static void cmdKillFunction(void)
{

	// Quito la tarea de la lista de tareas online para que no generen acciones que
	// interfieran con otras tareas de mantenimiento.
	// Pongo su wdg en un valor alto para no resetearme mientras estoy en servicio.

	pv_makeArgv();

	// KILL GPRSTX
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSTX\0"))) {
		vTaskSuspend( xHandle_tkGprs );
		// Dejo la flag de modem prendido para poder leer comandos
		GPRS_stateVars.modem_prendido = true;
		pub_control_watchdog_kick(WDG_GPRS, 0xFFFF);
		pv_snprintfP_OK();
		return;
	}

	// KILL GPRSRX
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSRX\0"))) {
		vTaskSuspend( xHandle_tkGprsRx );
		pub_control_watchdog_kick(WDG_GPRSRX, 0xFFFF);
		pv_snprintfP_OK();
		return;
	}

#ifdef SP5KV5_3CH
	// KILL OUTPUTS
	if (!strcmp_P( strupr(argv[1]), PSTR("OUTPUTS\0"))) {
		vTaskSuspend( xHandle_tkOutputs );
		pub_control_watchdog_kick(WDG_OUT, 0xFFFF);
		pv_snprintfP_OK();
		return;
	}
#endif /* SP5KV5_3CH */

	// KILL ANALOG
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOG\0"))) {
		vTaskSuspend( xHandle_tkAIn );
		pub_control_watchdog_kick(WDG_AIN, 0xFFFF);
		pv_snprintfP_OK();
		return;
	}

	// KILL DIGITAL
	if (!strcmp_P( strupr(argv[1]), PSTR("DIGITAL\0"))) {
		vTaskSuspend( xHandle_tkDigitalIn );
		pub_control_watchdog_kick(WDG_DIN, 0xFFFF);
		pv_snprintfP_OK();
		return;
	}

	pv_snprintfP_ERR();
	return;
}
/*------------------------------------------------------------------------------------*/
static void cmdStatusFunction(void)
{

RtcTimeType_t rtcDateTime;
uint16_t pos;
uint8_t channel;
StatBuffer_t pxFFStatBuffer;

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("\r\nSpymovil %s %s %dch %s %s\r\n\0"), SP5K_MODELO, SP5K_VERSION, NRO_ANALOG_CHANNELS, SP5K_REV, SP5K_DATE);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Last reset info
	pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("Wdg (0x%X"),wdgStatus.resetCause);
	if (wdgStatus.resetCause & 0x01 ) {
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR(" PORF"));
	}
	if (wdgStatus.resetCause & 0x02 ) {
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR(" EXTRF"));
	}
	if (wdgStatus.resetCause & 0x04 ) {
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR(" BORF"));
	}
	if (wdgStatus.resetCause & 0x08 ) {
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR(" WDRF"));
	}
	if (wdgStatus.resetCause & 0x10 ) {
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR(" JTRF"));
	}
	pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR(" )\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DlgId */
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("dlgid: %s\r\n\0"), systemVars.dlgId );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Fecha y Hora */
	RTC_read(&rtcDateTime);
	pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("rtc: %02d/%02d/%04d "),rtcDateTime.day,rtcDateTime.month, rtcDateTime.year );
	pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("%02d:%02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min, rtcDateTime.sec );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER */
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR(">Server:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* APN */
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  apn: %s\r\n\0"), systemVars.apn );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER IP:SERVER PORT */
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  server ip:port: %s:%s\r\n\0"), systemVars.server_ip_address,systemVars.server_tcp_port );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER SCRIPT */
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  server script: %s\r\n\0"), systemVars.serverScript );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER PASSWD */
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  passwd: %s\r\n\0"), systemVars.passwd );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// MODEM ---------------------------------------------------------------------------------------
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR(">Modem:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Modem band */
	pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  band: "));
	switch ( systemVars.gsmBand) {
	case 0:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("(900)"));
		break;
	case 1:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("(1800)"));
		break;
	case 2:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("dual band (900/1800)"));
		break;
	case 3:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("pcs (1900)"));
		break;
	case 4:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("gsm (850)"));
		break;
	case 5:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("dual band (1900/850)"));
		break;
	case 6:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("triband (900/1800/1900)"));
		break;
	case 7:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("triband (850/1800/1900)"));
		break;
	case 8:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("cuatriband (850/900/1800/1900)"));
		break;
	}
	pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* ROAMING */
	if ( systemVars.roaming == true ) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  roaming ON\r\n\0"));
	} else {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  roaming OFF\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DLGIP */
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  dlg ip: %s\r\n\0"), systemVars.dlg_ip_address );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CSQ */
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  signalQ: csq=%d, dBm=%d\r\n\0"), systemVars.csq, systemVars.dbm );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// GPRS STATE
	pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  state: "));
	switch (GPRS_stateVars.state) {
	case G_ESPERA_APAGADO:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("await_off\r\n"));
		break;
	case G_PRENDER:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("prendiendo\r\n"));
		break;
	case G_CONFIGURAR:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("configurando\r\n"));
		break;
	case G_MON_SQE:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("mon_sqe\r\n"));
		break;
	case G_GET_IP:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("ip\r\n"));
		break;
	case G_INIT_FRAME:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("init frame\r\n"));
		break;
	case G_DATA:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("data\r\n"));
		break;
	default:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("ERROR\r\n"));
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SYSTEM ---------------------------------------------------------------------------------------
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR(">System:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Memoria */
	FF_stat(&pxFFStatBuffer);
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  memory: wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d \r\n"), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

#ifdef SP5KV5_3CH
	// PWR SAVE:
	if ( systemVars.pwrSave.modo ==  modoPWRSAVE_OFF ) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  pwrsave=off\r\n\0"));
	} else {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  pwrsave=on start[%02d:%02d], end[%02d:%02d]\r\n\0"), systemVars.pwrSave.hora_start.hour, systemVars.pwrSave.hora_start.min, systemVars.pwrSave.hora_fin.hour, systemVars.pwrSave.hora_fin.min);
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  timerDial: [%lus]: %li\r\n\0"), systemVars.timerDial, pub_gprs_readTimeToNextDial() );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

#endif /* SP5KV5_3CH */

	/* Timers */
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  timerPoll: [%ds]\r\n\0"),systemVars.timerPoll );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Debug */
	pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  debug: "));
	if ( systemVars.debugLevel == D_NONE) {
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("none" ));
	} else if ( systemVars.debugLevel == D_ANALOG) {
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("data" ));
	} else if ( systemVars.debugLevel == D_GPRS) {
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("gprs" ));
	} else if ( systemVars.debugLevel == D_MEM) {
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("memory" ));

#ifdef SP5KV5_3CH
	} else if ( systemVars.debugLevel == D_OUTPUTS) {
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("outputs" ));
#endif /* SP5KV5_3CH */

	} else if ( systemVars.debugLevel == D_DIGITAL) {
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("digital" ));
	}
	FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

#ifdef SP5KV5_3CH

	// TERMINAL FIXED ON
	if ( systemVars.terminal_on ) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  terminal=FIX_ON\r\n\0"));
	} else {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  terminal=normal\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// OUTPUTS:
	switch( systemVars.outputs.modo ) {
	case OUT_OFF:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: OFF\r\n"));
		break;
	case OUT_CONSIGNA:
		switch( systemVars.outputs.consigna_aplicada ) {
		case CONSIGNA_DIURNA:
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: CONSIGNA [diurna] (c_dia=%02d:%02d, c_noche=%02d:%02d)\r\n"), systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
			break;
		case CONSIGNA_NOCTURNA:
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: CONSIGNA [nocturna] (c_dia=%02d:%02d, c_noche=%02d:%02d)\r\n"), systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
			break;
		default:
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: CONSIGNA [error] (c_dia=%02d:%02d, c_noche=%02d:%02d)\r\n"), systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
			break;
		}
		break;
	case OUT_NORMAL:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: NORMAL (out_A=%d, out_B=%d)\r\n"), systemVars.outputs.out_A, systemVars.outputs.out_B );
		break;
	default:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: ERROR(%d) (out_A=%d, out_B=%d)\r\n"), systemVars.outputs.modo, systemVars.outputs.out_A, systemVars.outputs.out_B );
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CONFIG */
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR(">Config:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Bateria
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  batt{0-15V}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );


	/* Configuracion de canales analogicos */
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  a%d{%d-%dmA/%.02f %.02f}(%s)\r\n\0"),channel, systemVars.Imin[channel],systemVars.Imax[channel],systemVars.Mmin[channel],systemVars.Mmax[channel], systemVars. aChName[channel] );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}
	/* Configuracion de canales digitales */
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  d%d{%.02f p/p} (%s)\r\n\0"), channel, systemVars.magPP[channel],systemVars.dChName[channel]);
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}
#endif /* SP5KV5_3CH */

#ifdef SP5KV5_8CH

	/* Configuracion de canales analogicos */
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  a%d{%d-%dmA/%.02f,%.02f} [%d],%s\r\n\0"),channel, systemVars.Imin[channel],systemVars.Imax[channel],systemVars.Mmin[channel],systemVars.Mmax[channel],systemVars.coef_calibracion[channel],systemVars.aChName[channel] );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	/* Configuracion de canales digitales */
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  d%d{%s}\r\n\0"),channel, systemVars.dChName[channel] );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}


#endif /* SP5KV5_8CH */

	/* VALUES --------------------------------------------------------------------------------------- */
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR(">Values:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	pub_analog_print_frame( pub_analog_get_data_frame_ptr() );

}
/*------------------------------------------------------------------------------------*/
static void cmdReadFunction(void)
{
char datetime[24];
bool retS = false;
uint8_t regValue;

	pv_makeArgv();

 	// TEST
 	if (!strcmp_P( strupr(argv[1]), PSTR("TEST\0"))) {
 		debug_test_printf();
 		return;
 	}

 	// WMK
 	if (!strcmp_P( strupr(argv[1]), PSTR("WMK\0"))) {
 		pub_debug_print_stack_watermarks();
 		return;
 	}

 	// WDT
 	if (!strcmp_P( strupr(argv[1]), PSTR("WDT\0"))) {
 		pub_debug_print_wdg_timers();
 		return;
 	}

	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		retS = EE_test_read( argv[2], cmd_printfBuff, argv[3] );
		if ( retS ) {
			// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
			FRTOS_snprintf_P( &cmd_printfBuff[atoi(argv[3])], sizeof(cmd_printfBuff), PSTR( "\r\n\0"));
			FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		if ( RTC_date_to_str( datetime, sizeof(datetime) ) != -1 ) {
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("OK\r\n%s\r\n\0"), datetime );
		} else {
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("ERROR\r\n\0"));
		}
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// MCP
	// read mcp 0|1|2 addr
	if (!strcmp_P( strupr(argv[1]), PSTR("MCP\0"))) {
		switch( atoi(argv[2] )) {
		case 0:
			retS = MCP_read( MCP0_ADDR, atoi(argv[3]), &regValue );
			break;
#ifdef SP5KV5_3CH
		case 1:
			retS = MCP_read( MCP1_ADDR, atoi(argv[3]), &regValue );
			break;
#endif /* SP5KV5_3CH */

		}

		if (retS ) {
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("OK\r\n[reg 0X%x]=[0X%x]\r\n\0"),atoi(argv[3]),regValue);
		} else {
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("ERROR\r\n\0"));
		}
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// DIN
	// read din 0|1
	if (!strcmp_P( strupr(argv[1]), PSTR("DIN\0"))) {
		pub_digital_read_Inputs( atoi(argv[2] ) );
		return;
	}

	// AN
	// read an id
	if (!strcmp_P( strupr(argv[1]), PSTR("AN\0"))) {
		pub_analog_read_Inputs( atoi(argv[2] ) );
		return;
	}

	// FRAME
	if (!strcmp_P( strupr(argv[1]), PSTR("FRAME\0"))) {
		pub_analog_read_frame( false);
		return;
	}

	// MEMORY
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))) {
		pv_readMemory();
		return;
	}

	// GPRS
	// read gprs (rsp,cts,dcd,ri)
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0")) ) {
		pv_cmd_rwGPRS(RD_CMD);
		return;
	}

#ifdef SP5KV5_3CH
 	// TERMSW
 	if (!strcmp_P( strupr(argv[1]), PSTR("TERMSW\0"))) {
 		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("OK\r\nTERMSW=%d\r\n\0"),IO_read_terminal_pin() );
 		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
 		return;

 	}

#endif /* SP5KV5_3CH */

#ifdef SP5KV5_8CH

	// INA
 	// read ina id regName
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0"))) {
		pub_analog_read_INA3221(argv[2], argv[3]);
		return;
	}

#endif /* SP5KV5_8CH */

	// CMD NOT FOUND
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
static void cmdConfigFunction(void)
{
bool retS = false;
uint8_t outputs_mode;

	pv_makeArgv();

	// DEFAULT (load default configuration)
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULTS\0"))) {
		pub_loadDefaults();
		return;
	}

	// SAVE
	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE\0"))) {
		retS = pub_saveSystemParams();
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// PASSWD
	if (!strcmp_P( strupr(argv[1]), PSTR("PASSWD\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.passwd, '\0', sizeof(systemVars.passwd));
			memcpy(systemVars.passwd, argv[2], sizeof(systemVars.passwd));
			systemVars.passwd[PASSWD_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// DLGID
	if (!strcmp_P( strupr(argv[1]), PSTR("DLGID\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memcpy(systemVars.dlgId, argv[2], sizeof(systemVars.dlgId));
			systemVars.dlgId[DLGID_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// APN
	if (!strcmp_P( strupr(argv[1]), PSTR("APN\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.apn, '\0', sizeof(systemVars.apn));
			memcpy(systemVars.apn, argv[2], sizeof(systemVars.apn));
			systemVars.apn[APN_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// ROAMING
	if (!strcmp_P( strupr(argv[1]), PSTR("ROAMING\0"))) {
		if (!strcmp_P( strupr(argv[2]),  PSTR("ON"))) { systemVars.roaming = true; }
		if (!strcmp_P( strupr(argv[2]),  PSTR("OFF"))) { systemVars.roaming = false; }
		pv_snprintfP_OK();
		return;
	}

	// GSMBAND:
	// Debo estar en modo service ya que para que tome el valor debe resetearse
	if (!strcmp_P( strupr(argv[1]), PSTR("GSMBAND\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			systemVars.gsmBand = atoi(argv[2]);
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER PORT
	if (!strcmp_P( strupr(argv[1]), PSTR("PORT\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.server_tcp_port, '\0', sizeof(systemVars.server_tcp_port));
			memcpy(systemVars.server_tcp_port, argv[2], sizeof(systemVars.server_tcp_port));
			systemVars.server_tcp_port[PORT_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER IP
	if (!strcmp_P( strupr(argv[1]), PSTR( "IP\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.server_ip_address, '\0', sizeof(systemVars.server_ip_address));
			memcpy(systemVars.server_ip_address, argv[2], sizeof(systemVars.server_ip_address));
			systemVars.server_ip_address[IP_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER SCRIPT
	if (!strcmp_P( strupr(argv[1]), PSTR("SCRIPT\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.serverScript, '\0', sizeof(systemVars.serverScript));
			memcpy(systemVars.serverScript, argv[2], sizeof(systemVars.serverScript));
			systemVars.serverScript[SCRIPT_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	/* DEBUGLEVEL */
	if (!strcmp_P( strupr(argv[1]), PSTR("DEBUG\0"))) {
		retS = pv_cmdWrDebugLevel(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CANALES ANALOGICOS
	// analog {0..8} aname imin imax mmin mmax
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOG\0"))) {
		pub_analog_config_channel( atoi(argv[2]), argv[3], argv[4], argv[5], argv[6], argv[7] );
		pv_snprintfP_OK();
		return;
	}

	// CANALES DIGITALES
	if (!strcmp_P( strupr(argv[1]), PSTR("DIGITAL\0"))) {
		pub_digital_config_channel( atoi(argv[2]),argv[3],argv[4]);
		pv_snprintfP_OK();
		return;
	}

	// TIMERPOLL
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERPOLL\0"))) {
		retS = pub_analog_config_timerpoll(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}


	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		retS = RTC_str_to_date(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

#ifdef SP5KV5_8CH
	// CSPAN
	// Coeficinte de correccion del conversor INA.
	if (!strcmp_P( strupr(argv[1]), PSTR("CSPAN\0"))) {
		pub_analog_config_cspan(argv[2], argv[3]);
		pv_snprintfP_OK();
		return;
	}


#endif /* SP5KV5_8CH */

#ifdef SP5KV5_3CH
	// TERMINAL FIXED
	if (!strcmp_P( strupr(argv[1]), PSTR("TERMINAL\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON"))) { systemVars.terminal_on = true; }
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))) { systemVars.terminal_on = false; }
		pv_snprintfP_OK();
		return;
	}

	// TIMERDIAL
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERDIAL\0"))) {
		retS = pub_configTimerDial(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// PWRSAVE
	if (!strcmp_P( strupr(argv[1]), PSTR("PWRSAVE\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR( "ON"))) { pub_configPwrSave ( modoPWRSAVE_ON, argv[3], argv[4] ); }
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))) { pub_configPwrSave ( modoPWRSAVE_OFF, argv[3], argv[4] ); }
		pv_snprintfP_OK();
		return;
	}

	// config outputs
	if (!strcmp_P( strupr(argv[1]), PSTR("OUTPUTS\0")) ) {
		outputs_mode = 0;
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0"))) {
			outputs_mode = 0;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("CONSIGNA\0"))) {
			outputs_mode = 1;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("NORMAL\0"))) {
			outputs_mode = 2;
		};

		pub_outputs_config( outputs_mode, argv[3], argv[4] );
		pv_snprintfP_OK();
		return;
	}
#endif /* SP5KV5_3CH */

	// CMD NOT FOUND
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;

}
/*------------------------------------------------------------------------------------*/
static void cmdWriteFunction(void)
{
bool retS = false;

	pv_makeArgv();

#ifdef SP5KV5_3CH
	// OUT 8814
	// write out sleep|reset|phase(A/B)|enable(A/B)| {0|1}
	//       out pulse (A/B) (+/-) (ms)
	//       out power {on|off}
	if (!strcmp_P( strupr(argv[1]), PSTR("OUT\0")) ) {
		pv_cmd_wrOUT8814();
		return;
	}

	// CONSIGNA
	// write consigna {diurna|nocturna}
	if (!strcmp_P( strupr(argv[1]), PSTR("CONSIGNA\0")) ) {

		if (!strcmp_P( strupr(argv[2]), PSTR("DIURNA\0")) ) {
			pub_output_set_consigna_diurna();
			pv_snprintfP_OK();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("NOCTURNA\0")) ) {
			pub_output_set_consigna_nocturna();
			pv_snprintfP_OK();
			return;
		}

		pv_snprintfP_ERR();
		return;
	}
#endif /* SP5KV5_3CH */

	// GPRS
	// write gprs pwr|sw|rts {on|off}
	// write gprs cmd {atcmd}
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0")) ) {
		pv_cmd_rwGPRS(WR_CMD);
		return;
	}

	// EE: write ee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		retS = EE_test_write( argv[2], argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

#ifdef SP5KV5_3CH
	// termPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("TERMPWR\0"))) {
		switch( atoi(argv[2]) ) {
		case 0:
			IO_term_pwr_off();
			retS = true;
			break;
		case 1:
			IO_term_pwr_on();
			retS = true;
			break;
		default:
			retS = false;
			break;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// sensorPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("SENSORPWR\0"))) {
		switch( atoi(argv[2]) ) {
		case 0:
			IO_sensor_pwr_off();
			retS = true;
			break;
		case 1:
			IO_sensor_pwr_on();
			retS = true;
			break;
		default:
			retS = false;
			break;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// analogPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOGPWR\0"))) {
		switch( atoi(argv[2]) ) {
		case 0:
			IO_analog_pwr_off();
			retS = true;
			break;
		case 1:
			IO_analog_pwr_on();
			retS = true;
			break;
		default:
			retS = false;
			break;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// Clear Q
	if (!strcmp_P( strupr(argv[1]), PSTR("CLEARQ\0"))) {

		if ( atoi(argv[2]) == 0 ) {
			IO_clear_Q0();
			vTaskDelay( ( TickType_t)( 1) );
			IO_set_Q0();
		}
		if ( atoi(argv[2]) == 1 ) {
			IO_clear_Q1();
			vTaskDelay( ( TickType_t)( 1) );
			IO_set_Q1();
		}
		return;
	}
#endif /* SP5KV5_3CH */

	// MCP
	// write mcp 0|1|2 addr value
	if (!strcmp_P( strupr(argv[1]), PSTR("MCP\0"))) {
		switch( atoi(argv[2] )) {
		case 0:
			retS = MCP_write( MCP0_ADDR, atoi(argv[3]), atoi(argv[4]) );
			break;

#ifdef SP5KV5_3CH
		case 1:
			retS = MCP_write( MCP1_ADDR, atoi(argv[3]), atoi(argv[4]) );
			break;
#endif /* SP5KV5_3CH */

		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CMD NOT FOUND
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
// FUNCIONES PRIVADAS
//-------------------------------------------------------------------------------------
bool pv_cmdWrDebugLevel(char *s)
{

	if ((!strcmp_P( strupr(s), PSTR("NONE")))) {
		systemVars.debugLevel = D_NONE;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("ANALOG")))) {
		systemVars.debugLevel = D_ANALOG;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("MEM")))) {
		systemVars.debugLevel = D_MEM;
		return(true);
	}

#ifdef SP5KV5_3CH
	if ((!strcmp_P( strupr(s), PSTR("OUTPUT")))) {
		systemVars.debugLevel = D_OUTPUTS;
		return(true);
	}
#endif /* SP5KV5_3CH */

	if ((!strcmp_P( strupr(s), PSTR("GPRS")))) {
		systemVars.debugLevel = D_GPRS;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("DIGITAL")))) {
		systemVars.debugLevel = D_DIGITAL;
		return(true);
	}

	return(false);
}
/*------------------------------------------------------------------------------------*/
static uint8_t pv_makeArgv(void)
{
// A partir de la linea de comando, genera un array de punteros a c/token
//
char *token = NULL;
char parseDelimiters[] = " ";
int i = 0;

	// inicialmente todos los punteros deben apuntar a NULL.
	memset(argv, 0, sizeof(argv) );

	// Genero los tokens delimitados por ' '.
	token = strtok(SP5K_CmdlineBuffer, parseDelimiters);
	argv[i++] = token;
	while ( (token = strtok(NULL, parseDelimiters)) != NULL ) {
		argv[i++] = token;
		if (i == 16) break;
	}
	return(( i - 1));
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_OK(void )
{
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("OK\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_ERR(void)
{
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("ERROR\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_readMemory(void)
{
	// Leemos la memoria e imprimo los datos.
	// El problema es que si hay muchos datos puede excederse el tiempo de watchdog y
	// resetearse el dlg.
	// Para esto, cada 32 registros pateo el watchdog.

StatBuffer_t pxFFStatBuffer;
frameData_t Aframe;
size_t bRead;
uint16_t rcds = 0;

	FF_seek();
	while(1) {
		bRead = FF_fread( &Aframe, sizeof(Aframe));

		if ( bRead == 0) {
			break;
		}

		if ( ( rcds++ % 32) == 0 ) {
			pub_control_watchdog_kick(WDG_CMD, WDG_CMD_TIMEOUT);
		}

		// imprimo
		FF_stat(&pxFFStatBuffer);
		FRTOS_snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff),  PSTR("RD:[%d/%d/%d][%d/%d] "), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

		pub_analog_print_frame(&Aframe);

	}
}
//------------------------------------------------------------------------------------
#ifdef SP5KV5_3CH
static void pv_cmd_wrOUT8814(void)
{
	// write out sleep|reset|phase(A/B)|enable(A/B)| {0|1}
	//       out pulse (A/B) (+/-) (ms)
	//       out power {on|off}

	// write out sleep 0,1
	if (!strcmp_P( strupr(argv[2]), PSTR("SLEEP\0")) ) {
		switch(atoi(argv[3])) {
		case 0:
			IO_clr_SLP();
			pv_snprintfP_OK();
			break;
		case 1:
			IO_set_SLP();
			pv_snprintfP_OK();
			break;
		default:
			pv_snprintfP_ERR();
		}
		return;
	}

	// write out reset 0,1
	if (!strcmp_P( strupr(argv[2]), PSTR("RESET\0")) ) {
		switch(atoi(argv[3])) {
		case 0:
			IO_clr_RES();
			pv_snprintfP_OK();
			break;
		case 1:
			IO_set_RES();
			pv_snprintfP_OK();
			break;
		default:
			pv_snprintfP_ERR();
		}
		return;
	}

	// write out phase (a/b) (0/1)
	if (!strcmp_P( strupr(argv[2]), PSTR("PHASE\0")) ) {
		switch (toupper(argv[3][0])) {
		case 'A':
			if (atoi(argv[4]) == 0) {
				IO_clr_PHA();
				pv_snprintfP_OK();
				return;
			}
			if (atoi(argv[4]) == 1) {
				IO_set_PHA();
				pv_snprintfP_OK();
				return;
			}
			pv_snprintfP_ERR();
			return;
		case 'B':
			if (atoi(argv[4]) == 0) {
				IO_clr_PHB();
				pv_snprintfP_OK();
				return;
			}
			if (atoi(argv[4]) == 1) {
				IO_set_PHB();
				pv_snprintfP_OK();
				return;
			}
			pv_snprintfP_ERR();
			return;
		}
	}

	// write out enable (a/b) (0/1)
	if (!strcmp_P( strupr(argv[2]), PSTR("ENABLE\0")) ) {
		switch (toupper(argv[3][0])) {
		case 'A':
			if (atoi(argv[4]) == 0) {
				IO_clr_ENA();
				pv_snprintfP_OK();
				return;
			}
			if (atoi(argv[4]) == 1) {
				IO_set_ENA();
				pv_snprintfP_OK();
				return;
			}
			pv_snprintfP_ERR();
			return;
		case 'B':
			if (atoi(argv[4]) == 0) {
				IO_clr_ENB();
				pv_snprintfP_OK();
				return;
			}
			if (atoi(argv[4]) == 1) {
				IO_set_ENB();
				pv_snprintfP_OK();
				return;
			}
			pv_snprintfP_ERR();
			return;
		}
	}

	// write out pulse (A/B) (+/-) (ms)
	if (!strcmp_P( strupr(argv[2]), PSTR("PULSE\0")) ) {
		DRV8814_test_pulse(argv[3],argv[4],argv[5]);
		pv_snprintfP_OK();
		return;
	}

	pv_snprintfP_ERR();
	return;

}
#endif /* SP5KV5_3CH */
//------------------------------------------------------------------------------------
static void pv_cmd_rwGPRS(uint8_t cmd_mode )
{


	if ( cmd_mode == WR_CMD ) {

		// write gprs (pwr|sw) {on|off}

		if (!strcmp_P( strupr(argv[2]), PSTR("PWR\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_modem_hw_pwr_on(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_modem_hw_pwr_off(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("SW\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_modem_sw_switch_high();
				pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_modem_sw_switch_low(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// write gprs redial
		if (!strcmp_P( strupr(argv[2]), PSTR("REDIAL\0")) ) {
			pub_gprs_redial();
			return;
		}
		// ATCMD
		// // write gprs cmd {atcmd}
		if (!strcmp_P(strupr(argv[2]), PSTR("CMD\0"))) {
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("%s\r\0"),argv[3] );
			FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
			FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
			pub_gprs_flush_RX_buffer();
			FreeRTOS_write( &pdUART0, cmd_printfBuff, sizeof(cmd_printfBuff) );
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("sent->%s\r\n\0"),argv[3] );
			FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
			return;
		}

		return;
	}

	if ( cmd_mode == RD_CMD ) {
		// read gprs (rsp)

			// ATCMD
			// read gprs rsp
			if (!strcmp_P(strupr(argv[2]), PSTR("RSP\0"))) {
				//p = FreeRTOS_UART_getFifoPtr(&pdUART_GPRS);
				FreeRTOS_write( &pdUART1, "rx->", sizeof("rx->")  );
				pub_gprs_print_RX_Buffer();
				return;
			}

			// DCD
/*			if (!strcmp_P( strupr(argv[2]), PSTR("DCD\0")) ) {
				pin = IO_read_DCD();
				FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("DCD=%d\r\n\0"),pin);
				CMD_write(  cmd_printfBuff, sizeof(cmd_printfBuff) );
				pv_snprintfP_OK();
				return;
			}
*/
			pv_snprintfP_ERR();
			return;
	}

}
//------------------------------------------------------------------------------------
