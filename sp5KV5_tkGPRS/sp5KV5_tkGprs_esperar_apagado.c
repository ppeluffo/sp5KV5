/*
 * sp5KV5_tkGprs_esperar_apagado.c
 *
 *  Created on: 28 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV5_tkGprs.h"


static int32_t waiting_time;

static bool pv_check_inside_pwrSave(void);

//------------------------------------------------------------------------------------
bool gprs_esperar_apagado(void)
{
	// Calculo el tiempo a esperar y espero. El intervalo no va a considerar el tiempo
	// posterior de proceso.

BaseType_t xResult;
uint32_t ulNotifiedValue;
static bool starting_flag = true;
bool exit_flag = false;

// Entry:

	// Secuencia para apagar el modem y dejarlo en modo low power.
	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS_PWROFF:: Apago modem\r\n\0"), u_now());
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	GPRS_stateVars.modem_prendido = false;
	strncpy_P(systemVars.dlg_ip_address, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;
	// Para que no consuma
	IO_modem_hw_pwr_off();
	IO_modem_sw_switch_high();
	IO_modem_hw_pwr_off();

	// Cuando arranco ( la primera vez) solo espero 10s y disco por primera vez
	if ( starting_flag ) {
		starting_flag = false;
		waiting_time = 10;
		goto LOOP;
	}

	// Calculo el tiempo que voy a tener que estar esperando
	// NORMAL:
	switch ( systemVars.wrkMode ) {
	case WK_NORMAL:
		// Depende en que modo de pwr estoy.
		if ( systemVars.pwrMode == PWR_CONTINUO ) {
			waiting_time = 30;
		}

		if ( ( systemVars.pwrMode == PWR_DISCRETO ) && ( pv_check_inside_pwrSave() == true ) ) {
			waiting_time = 600;
			if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
				snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS_PWRSAVE::\r\n\0"), u_now() );
				FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
			}
		}

		if ( ( systemVars.pwrMode == PWR_DISCRETO ) && ( pv_check_inside_pwrSave() == false ) ) {
			waiting_time = systemVars.timerDial;
			// Controlo que nunca disque en menos de 10 minutos.
			if ( waiting_time < 600 ) {
				waiting_time = 600;
			}
		}
		break;
	case WK_SERVICE:
		// Debo dejar apagado el modem. Si lo necesito lo prendo por cmdline
		waiting_time = 0xFFFF;
		break;
	case WK_MONITOR_FRAME:
		// Debo dejar apagado el modem
		waiting_time = 0xFFFF;
		break;
	case WK_MONITOR_SQE:
		// Debo prender cuanto antes ( 10s ) para ir a monitorear el sqe
		waiting_time = 10;
		break;
	default:
		waiting_time = 15;
		break;
	}

LOOP:
	// Aqui genero el loop de espera.

	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS_WAIT:: %lu s\r\n\0"), u_now(), waiting_time );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// Larga espera. Monitoreo las señales.
	while (  waiting_time-- > 0) {

		// Analizo las señales
		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 250 / portTICK_RATE_MS ) );
		if ( xResult == pdTRUE ) {

			if ( ( ulNotifiedValue & TK_PARAM_RELOAD ) != 0 ) {		// Mensaje de reload configuration.
				exit_flag = bool_RESTART ;							// Retorna y hace que deba ir a RESTART y leer la nueva configuracion
				goto EXIT;
			} else if ( ( ulNotifiedValue & TK_REDIAL ) != 0 ) {  	// Mensaje de read frame desde el cmdLine.
				exit_flag = bool_CONTINUAR ;						// Retorna y avanzo para discar rapido
				goto EXIT;
			} else if ( ( ulNotifiedValue & TK_TILT ) != 0 ) {  	// Mensaje de tilt desde tkControl.
				exit_flag = bool_CONTINUAR ;						// Retorna y avanzo para discar rapido
				goto EXIT;
			}
		}

		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

	}

	// Termine la espera bien.
	exit_flag = bool_CONTINUAR;

	// Exit area:
EXIT:
	// No espero mas y salgo del estado prender.
	waiting_time = -1;
	return(exit_flag);

}
//------------------------------------------------------------------------------------
static bool pv_check_inside_pwrSave(void)
{
	// Calculo el waiting time en modo DISCRETO evaluando si estoy dentro
	// del periodo de pwrSave.
	// En caso afirmativo, seteo el tiempo en 10mins ( 600s )
	// En caso negativo, lo seteo en systemVars.timerDial

RtcTimeType_t rtcDateTime;
uint16_t now, pwr_save_start, pwr_save_end ;

	if ( systemVars.pwrSave == modoPWRSAVE_OFF ) {
		// PWR SAVE DESACTIVADO
		return(false);

	} else {

		// Estoy con PWR SAVE ACTIVADO
		RTC_read(&rtcDateTime);
		now = rtcDateTime.hour * 60 + rtcDateTime.min;
		pwr_save_start = systemVars.pwrSaveStartTime.hour * 60 + systemVars.pwrSaveStartTime.min;
		pwr_save_end = systemVars.pwrSaveEndTime.hour * 60 + systemVars.pwrSaveEndTime.min;

		if ( pwr_save_start < pwr_save_end ) {
			// Caso A:
			if ( now <= pwr_save_start ) { return(false); }
			// Caso B:
			if ( ( pwr_save_start <= now ) && ( now <= pwr_save_end )) { return(true); }
			// Caso C:
			if ( now > pwr_save_end ) { return(false); }
		}

		if (  pwr_save_end < pwr_save_start ) {
			// Caso A:
			if ( now <=  pwr_save_end ) { return(true); }
			// Caso B:
			if ( ( pwr_save_end <= now ) && ( now <= pwr_save_start )) { return(false); }
			// Caso C:
			if ( now > pwr_save_start ) { return(true); }
		}
	}

	return(false);
}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//----------------------------------------------------------------------------------------
int32_t u_readTimeToNextDial(void)
{
	return(waiting_time);
}
//----------------------------------------------------------------------------------------
