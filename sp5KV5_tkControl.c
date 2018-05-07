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
#define WDG_CTL_TIMEOUT	10

const char string_0[] PROGMEM = "CMD";
const char string_1[] PROGMEM = "CTL";
const char string_2[] PROGMEM = "DIN";
const char string_3[] PROGMEM = "AIN";
const char string_4[] PROGMEM = "OUT";
const char string_5[] PROGMEM = "GTX";
const char string_6[] PROGMEM = "GRX";

const char * const wdg_names[] PROGMEM = { string_0, string_1, string_2, string_3, string_4, string_5, string_6 };

//------------------------------------------------------------------------------------
void tkControl(void * pvParameters)
{

( void ) pvParameters;

	// Aqui solo controlo la terminal por lo que no me importa tanto el watchdog.
	// Lo controlo en LedTiltWdg

	pv_tkControl_init();
	pv_init_show_reset_cause();

	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("-----------------\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("starting tkControl..\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	//debug_print_self_stack_watermark(4);
	// CTL_STACK_SIZE = 512w, HWM=49w

	// Al comienzo leo este handle para asi usarlo para leer el estado de los stacks.
	xHandle_idle = xTaskGetIdleTaskHandle();

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

	// Si algun WDG no se borro, me reseteo
	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	for ( wdg = 0; wdg < NRO_WDGS; wdg++ ) {
		if ( --watchdog_timers[wdg] == 0 ) {
			FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("CTL: WDG TO(%d) !!\r\n\0"),wdg);
			FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
			while(1);		// Me reseteo por watchdog
		}
	}

	xSemaphoreGive( sem_SYSVars );

	// Cada ciclo reseteo el wdg para que no expire.
	wdt_reset();
}
//------------------------------------------------------------------------------------
static void pv_check_terminal(void)
{
static uint8_t timer_off = 30;
static uint8_t pinAnt = 1;
uint8_t pin;

	// Si estamos en modo continuo ( systemVars.timerDial = 0 ) la terminal queda prendida.

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

	// Normalmente....
	if ( systemVars.timerDial == 0 ) {
		// Si estoy en modo continuo no apago.
		pin = 1;	// Es como si tubiese el switch activado.
	} else {
		// Modo MANUAL ( por switch )
		pin = IO_read_terminal_pin();
	}

	// Transicion 1-> 0: APAGO
	if ( ( pinAnt == 1) && ( pin == 0 ) ) {
		FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("CTL: Terminal going off..\r\n\0"));
		FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
		vTaskDelay( ( TickType_t)( 1500 / portTICK_RATE_MS ) );
		IO_term_pwr_off();
		f_terminalStatus = T_APAGADA;
		pub_uarts_ctl(TERM_APAGAR);
		pinAnt = pin;
		return;
	}

	// Transicion 0-> 1: PRENDO
	if ( ( pinAnt == 0) && ( pin == 1 ) ) {
		IO_term_pwr_on();
		f_terminalStatus = T_PRENDIDA;
		pub_uarts_ctl(TERM_PRENDER);
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

     	if ( pub_gprs_modem_prendido() ) {
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

	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("CTL: Daily Reset !!\r\n\0") );
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
uint16_t recSize;
uint8_t wdg;
bool load_system_params;

	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	MCP_init(0);				// Esto prende la terminal.
	MCP_init(1);
	IO_term_pwr_on();
	f_terminalStatus = T_PRENDIDA;
	pub_uarts_ctl(TERM_PRENDER);


	load_system_params = false;
	// Load systemVars
	if  ( pub_loadSystemParams() == true ) {
		load_system_params = true;
	} else {
		pub_loadDefaults();
		pub_saveSystemParams();
		load_system_params = false;
	}

	// Configuro el ID en el bluetooth: debe hacerse antes que nada
	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("AT+NAME%s\r\n"),systemVars.dlgId);
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );

	if  ( load_system_params ) {
		FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Load config OK.\r\n\0") );
	} else {
		FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Load config ERROR: defaults !!\r\n\0") );
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Inicializo la memoria EE ( fileSysyem)
	ffRcd = FF_fopen();
	FF_stat(&pxFFStatBuffer);
	if ( pxFFStatBuffer.errno != pdFF_ERRNO_NONE ) {
		FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("FSInit ERROR (%d)[%d]\r\n\0"),ffRcd, pxFFStatBuffer.errno);
	} else {
		FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("FSInit OK\r\nMEMsize=%d, wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d\r\n\0"),FF_MAX_RCDS, pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Tamanio de registro de memoria
	recSize = sizeof(frameData_t);
	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("RCD size %d bytes.\r\n\0"),recSize);
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	pos = FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Modules:: BASIC\0"));
	pos += FRTOS_snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR("+PRESION\0"));
	pos += FRTOS_snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR("+CONSIGNA\0"));
	pos += FRTOS_snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR("\r\n"));
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

	pos = FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Init code (0x%X"),wdgStatus.resetCause);
	if (wdgStatus.resetCause & 0x01 ) {
		pos += FRTOS_snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" PORF"));
	}
	if (wdgStatus.resetCause & 0x02 ) {
		pos += FRTOS_snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" EXTRF"));
	}
	if (wdgStatus.resetCause & 0x04 ) {
		pos += FRTOS_snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" BORF"));
	}
	if (wdgStatus.resetCause & 0x08 ) {
		pos += FRTOS_snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" WDRF"));
	}
	if (wdgStatus.resetCause & 0x10 ) {
		pos += FRTOS_snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" JTRF"));
	}
	pos += FRTOS_snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" )\r\n\0"));
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
void pub_debug_print_wdg_timers(void)
{

uint8_t wdg;
char buffer[10];

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	for ( wdg = 0; wdg < NRO_WDGS; wdg++ ) {
		memset(buffer,'\0', 10);
		strcpy_P(buffer, (PGM_P)pgm_read_word(&(wdg_names[wdg])));
		FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("%d(%s)->%d \r\n\0"),wdg,buffer,watchdog_timers[wdg]);
		FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	}

	xSemaphoreGive( sem_SYSVars );

	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

}
//------------------------------------------------------------------------------------
void pub_debug_print_stack_watermarks(void)
{

UBaseType_t uxHighWaterMark;

	// tkIdle
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_idle );
	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("IDLE:%03d,%03d,[%03d]\r\n\0"),configMINIMAL_STACK_SIZE,uxHighWaterMark,(configMINIMAL_STACK_SIZE - uxHighWaterMark)) ;
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// tkCmd
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkCmd );
	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("CMD: %03d,%03d,[%03d]\r\n\0"),tkCmd_STACK_SIZE,uxHighWaterMark,(tkCmd_STACK_SIZE - uxHighWaterMark)) ;
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// tkControl
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkControl );
	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("CTL: %03d,%03d,[%03d]\r\n\0"),tkControl_STACK_SIZE,uxHighWaterMark, (tkControl_STACK_SIZE - uxHighWaterMark));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// tkDigital
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkDigitalIn );
	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("DIN: %03d,%03d,[%03d]\r\n\0"),tkDigitalIn_STACK_SIZE,uxHighWaterMark, ( tkDigitalIn_STACK_SIZE - uxHighWaterMark));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// tkAnalog
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkAIn );
	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("AIN: %03d,%03d,[%03d]\r\n\0"),tkAIn_STACK_SIZE,uxHighWaterMark, ( tkAIn_STACK_SIZE - uxHighWaterMark));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// tkOutputs
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkOutputs );
	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("OUT: %03d,%03d,[%03d]\r\n\0"),tkOutputs_STACK_SIZE, uxHighWaterMark, ( tkOutputs_STACK_SIZE - uxHighWaterMark));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	//kGprsTX
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkGprs );
	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("GTX: %03d,%03d,[%03d]\r\n\0"),tkGprs_STACK_SIZE, uxHighWaterMark, ( tkGprs_STACK_SIZE - uxHighWaterMark));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// tkGprsRX
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkGprsRx );
	FRTOS_snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("GRX: %03d,%03d,[%03d]\r\n\0"),tkGprsRx_STACK_SIZE,uxHighWaterMark, ( tkGprsRx_STACK_SIZE - uxHighWaterMark));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

}
//------------------------------------------------------------------------------------

