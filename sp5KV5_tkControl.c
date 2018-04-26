/*
 * sp5KV3_tkControl.c
 *
 *  Created on: 7/4/2015
 *      Author: pablo
 *
 *  Tareas de control generales del SP5K
 *  - Recibe un mensaje del timer del led para indicar si debe prender o apagarlo.
 */

#include "sp5KV5.h"
#include "sp5KV5_tkGPRS/sp5KV5_tkGprs.h"

static char ctl_printfBuff[CHAR128];

//static bool f_tilt_alarmFired = false;
static t_terminalStatus f_terminalStatus;

static void pv_tkControl_init(void);
static void pv_init_show_reset_cause(void);

static void pv_check_terminal(void);
static void pv_check_leds(void);
static void pv_check_daily_reset(void);
static void pv_check_wdg(void);

static uint16_t watchdog_timers[NRO_WDGS];

// La tarea pasa por el mismo lugar c/1s.
#define WDG_CTL_TIMEOUT	5
//------------------------------------------------------------------------------------
void tkControl(void * pvParameters)
{

( void ) pvParameters;

	// Aqui solo controlo la terminal por lo que no me importa tanto el watchdog.
	// Lo controlo en LedTiltWdg

	pv_tkControl_init();
	pv_init_show_reset_cause();

	FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"-----------------\r\n\0");
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"starting tkControl..\r\n\0");
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Loop
    for( ;; )
    {

    	pub_control_watchdog_kick(WDG_CTL, WDG_CTL_TIMEOUT);

	   	// Espero 1 segundo para revisar todo.
        vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

        // Reviso los sistemas perifericos.
        pv_check_terminal();
        pv_check_leds();
        pv_check_daily_reset();
        pv_check_wdg();

    }

}
//------------------------------------------------------------------------------------
static void pv_check_wdg(void)
{
	// Cada tarea periodicamente reinicia su wdg timer.
	// Esta tarea los decrementa c/segundo.
	// Si alguno llego a 0 es que la tarea se colgo y entonces se reinicia el sistema.

uint8_t wdg;

	// Si algun WDG no se borro, me reeseteo
	for ( wdg = 0; wdg < NRO_WDGS; wdg++ ) {
		if ( --watchdog_timers[wdg] == 0 ) {
			while(1);
		}
	}

	wdt_reset();
}
//------------------------------------------------------------------------------------
static void pv_check_terminal(void)
{
static uint8_t timer_off = 30;
static uint8_t pinAnt = 1;
uint8_t pin;

	// Inicialmente espero que pase timer_off para comenzar a evaluar
	// la situacion de la terminal. Durante este tiempo inicial esta prendida
	if ( timer_off > 0 ) {
		pinAnt = 1;		// Para que al cambiar de modo a OFF detecte una transicion.
		timer_off--;
		return;
	}

	// Si configure en consola la terminal esta para que quede prendida, lo dejo asi y salgo
	if (  systemVars.terminal_on ) {
		pinAnt = 1;	// Para que al cambiar de modo a OFF detecte una transicion.
		return;
	}

	// Modo MANUAL ( por switch )
	pin = IO_read_terminal_pin();

	// Transicion 1-> 0: APAGO
	if ( ( pinAnt == 1) && ( pin == 0 ) ) {
		FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"CTL: Terminal going off..\r\n\0");
		FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
		vTaskDelay( ( TickType_t)( 1500 / portTICK_RATE_MS ) );
		IO_term_pwr_off();
		f_terminalStatus = T_APAGADA;
		u_uarts_ctl(TERM_APAGAR);
		pinAnt = pin;
		return;
	}

	// Transicion 0-> 1: PRENDO
	if ( ( pinAnt == 0) && ( pin == 1 ) ) {
		IO_term_pwr_on();
		f_terminalStatus = T_PRENDIDA;
		u_uarts_ctl(TERM_PRENDER);
		pinAnt = pin;
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_check_leds(void)
{

static uint8_t count = 3;

	// Los leds flashean c/3s solo si la terminal esta prendida.
	// Siempre los apago para no correr riesgo que queden prendidos.

	if ( --count > 0 ) {
		return;
	}

	count = 3;

    if ( pub_control_terminal_is_on() == true ) {
    	// Prendo.
    	IO_set_led_KA_logicBoard();				// Led de KA de la placa logica
    	IO_set_led_KA_analogBoard();			// Idem. analog board

     	if ( u_modem_prendido() ) {
    		IO_set_led_MODEM_analogBoard();
    	}

   	}

   	// no es necesario ya que lo que demora las MCP son suficientes.
   	//vTaskDelay( 1 );

   	// Apago
   	IO_clear_led_KA_logicBoard();
    IO_clear_led_KA_analogBoard();
    IO_clear_led_MODEM_analogBoard();

 }
//------------------------------------------------------------------------------------
static void  pv_check_daily_reset(void)
{
	// Todos los dias debo resetearme para restaturar automaticamente posibles
	// problemas.

static uint32_t ticks_to_reset = 86400; // Segundos en 1 dia.

	while ( --ticks_to_reset > 0 ) {
		return;
	}

	FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"CTL: Daily Reset !!\r\n\0" );
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	wdt_enable(WDTO_30MS);
	while(1) {}

}
//------------------------------------------------------------------------------------
// FUNCIONES DE INIT
//------------------------------------------------------------------------------------
static void pv_tkControl_init(void)
{

uint8_t ffRcd;
StatBuffer_t pxFFStatBuffer;
uint16_t pos;
int8_t loadParamStatus = false;
uint16_t recSize;
uint8_t wdg;

	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	MCP_init(0);				// Esto prende la terminal.
	MCP_init(1);
	IO_term_pwr_on();
	f_terminalStatus = T_PRENDIDA;
	u_uarts_ctl(TERM_PRENDER);

	// Load systemVars
	if  ( u_loadSystemParams() == true ) {
		loadParamStatus = true;
	} else {
		u_loadDefaults();
		u_saveSystemParams();
		loadParamStatus = false;
	}

	// Configuro el ID en el bluetooth: debe hacerse antes que nada
	FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"AT+NAME%s\r\n",systemVars.dlgId);
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );

	// Mensaje de load Status.
	if ( loadParamStatus ) {
		FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"Load config OK.\r\n\0" );
	} else {
		FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"Load config ERROR: defaults !!\r\n\0" );
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Inicializo la memoria EE ( fileSysyem)
	ffRcd = FF_fopen();
	FF_stat(&pxFFStatBuffer);
	if ( pxFFStatBuffer.errno != pdFF_ERRNO_NONE ) {
		FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"FSInit ERROR (%d)[%d]\r\n\0",ffRcd, pxFFStatBuffer.errno);
	} else {
		FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"FSInit OK\r\nMEMsize=%d, wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d\r\n\0",FF_MAX_RCDS, pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Tamanio de registro de memoria
	recSize = sizeof(frameData_t);
	FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"RCD size %d bytes.\r\n\0",recSize);
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	pos = FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"Modules:: BASIC\0");
	pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),"+PRESION\0");
	pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),"+CONSIGNA\0");
	pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),"\r\n");
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Inicializo los watchdogs a 10s para dar tiempo a todas las tareas que arranquen y se configuren.
	for (wdg=0; wdg<NRO_WDGS;wdg++) {
		watchdog_timers[wdg] = 10;
	}

	// Habilito al resto de las tareas a arrancar.
	startTask = true;

}
//------------------------------------------------------------------------------------
static void pv_init_show_reset_cause(void)
{
uint8_t pos;

	// Muestro la razon del ultimo reseteo

	pos = FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"Init code (0x%X",wdgStatus.resetCause);
	if (wdgStatus.resetCause & 0x01 ) {
		pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff)," PORF");
	}
	if (wdgStatus.resetCause & 0x02 ) {
		pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff)," EXTRF");
	}
	if (wdgStatus.resetCause & 0x04 ) {
		pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff)," BORF");
	}
	if (wdgStatus.resetCause & 0x08 ) {
		pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff)," WDRF");
	}
	if (wdgStatus.resetCause & 0x10 ) {
		pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff)," JTRF");
	}
	pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff)," )\r\n\0");
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS DE USO GENERAL
//------------------------------------------------------------------------------------
bool pub_control_terminal_is_on(void)
{
	if ( f_terminalStatus == T_PRENDIDA ) {
		return(true);
	} else {
		return(false);
	}
}
//------------------------------------------------------------------------------------
void pub_control_watchdog_kick(uint8_t taskWdg, uint16_t timeout_in_secs )
{
	// Reinicia el watchdog de la tarea taskwdg con el valor timeout.
	// timeout es uint16_t por lo tanto su maximo valor en segundos es de 65536 ( 18hs )

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	watchdog_timers[taskWdg] = timeout_in_secs;

	xSemaphoreGive( sem_SYSVars );
}
//------------------------------------------------------------------------------------
void pub_print_wdg_timers(void)
{

uint8_t wdg;

	for ( wdg = 0; wdg < NRO_WDGS; wdg++ ) {
		FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"[%d][%05d] \0",wdg,watchdog_timers[wdg]);
		FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	}
	FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"\r\n\0");
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

}

//------------------------------------------------------------------------------------
