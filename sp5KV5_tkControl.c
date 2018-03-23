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

static bool f_tilt_alarmFired = false;
static t_terminalStatus f_terminalStatus;

static void pv_tkControl_init(void);
static void pv_init_show_reset_cause(void);

static void pv_check_terminal(void);
static void pv_check_leds(void);
static void pv_check_tilt(void);
static void pv_check_exit_modoService(void);
static void pv_check_daily_reset(void);
static void pv_check_wdg(void);

//------------------------------------------------------------------------------------
void tkControl(void * pvParameters)
{

( void ) pvParameters;

	// Aqui solo controlo la terminal por lo que no me importa tanto el watchdog.
	// Lo controlo en LedTiltWdg

	pv_tkControl_init();
	pv_init_show_reset_cause();

	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("-----------------\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("starting tkControl..\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Loop
    for( ;; )
    {

    	u_kick_Wdg(WDG_CTL);

	   	// Espero 1 segundo para revisar todo.
        vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

        // Reviso los sistemas perifericos.
        pv_check_terminal();
        pv_check_tilt();
        pv_check_leds();
        pv_check_exit_modoService();
        pv_check_daily_reset();
        pv_check_wdg();

    }

}
//------------------------------------------------------------------------------------
static void pv_check_wdg(void)
{
	// Cada tarea periodicamente pone su wdg flag en 0. Esto hace que al chequearse c/3s
	// deban estar todas en 0 para asi resetear el wdg del micro.

static u08 l_timer = 20;

	// Espero 10s sin resetearme
	if (l_timer-- > 0 )
		wdt_reset();
		return;

	// Si algun WDG no se borro, me reeseteo
	if ( systemWdg != 0 ) {
		while(1);
	}

	// Reincializo el sistema
	systemWdg = WDG_CTL + WDG_CMD + WDG_DIN + WDG_AIN + WDG_OUT;
	l_timer = 20;
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
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("%s CTL::term: Terminal going off ..\r\n\0"), u_now() );
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

    if ( u_terminal_is_on() == true ) {
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
static void pv_check_tilt(void)
{

	// Cuando detecta un tilt, queda alarmado y genera un dial.
	// Solo se borra la flag con un reset !!!

static uint8_t tilt_ant = 0;

   	if ( systemVars.tiltEnabled == false ) {
    	return;
   	}

   	// Deteccion que se movio.
   	if ( ( tilt_ant == 0 ) && ( IO_read_tilt_pin() == 1) ) {
    	// Se movio. Disparo un llamado y quedo alarmado.
    	if ( ! f_tilt_alarmFired ) {
    		f_tilt_alarmFired = true;
    		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("%s CTL::tilt: Flood alarm fired..\r\n"), u_now() );
   			FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

   			// Mando un mensaje a tkGPRS para que disque inmediatamente
   			while ( xTaskNotify(xHandle_tkGprsRx, TK_TILT , eSetBits ) != pdPASS ) {
   				vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
   			}

   		}
   	}
 }
//------------------------------------------------------------------------------------
static void pv_check_exit_modoService(void)
{
	// Cuando estoy en modo service, debo salir automaticamente a los 30minutos

static uint16_t sec_auto_exit = 1800;

	// En modo service, monitor_frame, monitor_sqe cuento.
	if ( systemVars.wrkMode != WK_NORMAL ) {
		sec_auto_exit--;
		if ( sec_auto_exit == 0 ) {
			snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("%s CTL::autoexit: Automatic exit of service mode..\r\n\0"), u_now() );
			FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
			vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
			// RESET
			u_reset();
		}

	} else {
		// En WRK_NORMAL reseteo el timer.
		sec_auto_exit = 1800;
	}

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

	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("%s CTL::reset: Daily Reset !!\r\n\0"), u_now() );
	u_debugPrint( D_BASIC, ctl_printfBuff, sizeof(ctl_printfBuff) );
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
	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("AT+NAME%s\r\n"),systemVars.dlgId);
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );

	// Mensaje de load Status.
	if ( loadParamStatus ) {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Load config OK.\r\n\0") );
	} else {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Load config ERROR: defaults !!\r\n\0") );
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Inicializo la memoria EE ( fileSysyem)
	ffRcd = FF_fopen();
	FF_stat(&pxFFStatBuffer);
	if ( pxFFStatBuffer.errno != pdFF_ERRNO_NONE ) {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("FSInit ERROR (%d)[%d]\r\n\0"),ffRcd, pxFFStatBuffer.errno);
	} else {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("FSInit OK\r\nMEMsize=%d, wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d\r\n\0"),FF_MAX_RCDS, pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Tamanio de registro de memoria
	recSize = sizeof(frameData_t);
	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("RCD size %d bytes.\r\n\0"),recSize);
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	pos = snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Modules:: BASIC\0"));
	pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR("+PRESION\0"));
	pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR("+CONSIGNA\0"));
	pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR("\r\n"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Habilito al resto de las tareas a arrancar.
	startTask = true;

}
//------------------------------------------------------------------------------------
static void pv_init_show_reset_cause(void)
{
uint8_t pos;

	// Muestro la razon del ultimo reseteo

	pos = snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Init code (0x%X"),wdgStatus.resetCause);
	if (wdgStatus.resetCause & 0x01 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" PORF"));
	}
	if (wdgStatus.resetCause & 0x02 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" EXTRF"));
	}
	if (wdgStatus.resetCause & 0x04 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" BORF"));
	}
	if (wdgStatus.resetCause & 0x08 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" WDRF"));
	}
	if (wdgStatus.resetCause & 0x10 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" JTRF"));
	}
	pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" )\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS DE USO GENERAL
//------------------------------------------------------------------------------------
bool u_tilt_alarmFired(void)
{
	return (f_tilt_alarmFired);
}
//------------------------------------------------------------------------------------
bool u_terminal_is_on(void)
{
	if ( f_terminalStatus == T_PRENDIDA ) {
		return(true);
	} else {
		return(false);
	}
}
//------------------------------------------------------------------------------------


