/*
 * sp5KV5_tkGprs_esperar_apagado.c
 *
 *  Created on: 28 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV5_tkGprs.h"


static int32_t waiting_time;

static bool pv_check_inside_pwrSave(void);
static void pv_calcular_tiempo_espera(void);
static bool pv_procesar_signals_espera( bool *exit_flag );

static bool starting_flag = true;

//------------------------------------------------------------------------------------
bool gprs_esperar_apagado(void)
{
	// Calculo el tiempo a esperar y espero. El intervalo no va a considerar el tiempo
	// posterior de proceso.

bool exit_flag = false;

// Entry:

	GPRS_stateVars.state = G_ESPERA_APAGADO;
	u_uarts_ctl(MODEM_APAGAR);

	// Secuencia para apagar el modem y dejarlo en modo low power.
	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::wait: Apago modem\r\n\0"), u_now());
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	GPRS_stateVars.modem_prendido = false;
	strncpy_P(systemVars.dlg_ip_address, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;
	// Para que no consuma

	// Apago por SW.
	IO_modem_sw_switch_high();
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	IO_modem_sw_switch_low();
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	IO_modem_sw_switch_high();

	// Apago por HW.
	IO_modem_hw_pwr_off();
	vTaskDelay( (portTickType)( 2000 / portTICK_RATE_MS ) );
	IO_modem_hw_pwr_on();

	pv_calcular_tiempo_espera();

	// Espera
	do {

		// Espero 1s.
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		waiting_time--;

		// Analizo si termine el periodo y debo salir
		if ( waiting_time == 0 ) {

			// 1: Si estoy en un arranque salgo y listo.
			if ( starting_flag == true ) {
				starting_flag = false;
				break;
			} else {
				// 2: Estoy dentro de pwrSave ? Espero 10 minutos mas.
				if ( pv_check_inside_pwrSave() ) {
					waiting_time = 600;
				}
			}
		}

		// PROCESO LAS SEÑALES
		if ( pv_procesar_signals_espera( &exit_flag )) {
			// Si recibi alguna senal, debo salir.
			goto EXIT;
		}

	} while (waiting_time > 0);

	//
	exit_flag = bool_CONTINUAR;

EXIT:

	// No espero mas y salgo del estado prender.
	waiting_time = -1;
	return(exit_flag);

}
//------------------------------------------------------------------------------------
static void pv_calcular_tiempo_espera(void)
{

	// Cuando arranco ( la primera vez) solo espero 10s y disco por primera vez
	if ( starting_flag ) {
		waiting_time = 10;
		return;
	}

	// Calculo el tiempo que voy a tener que estar esperando
	// NORMAL:
	switch ( systemVars.wrkMode ) {
	case WK_NORMAL:
		if ( MODO_DISCRETO ) {
			waiting_time = systemVars.timerDial;
			// Controlo que nunca disque en menos de 10 minutos.
			if ( waiting_time < 600 ) {
				//systemVars.timerDial = 600;
				waiting_time = 600;
			}
		} else {
			waiting_time = 30;
		}
		break;

	case WK_SERVICE:
		// Debo dejar apagado el modem. Si lo necesito lo prendo por cmdline
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

	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::wait: %lu s\r\n\0"), u_now(), waiting_time );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

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
bool insidePwrSave_flag = false;

	if ( (systemVars.debugLevel & D_GPRS ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::wait: check pwrsave\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// Estoy en modo PWR_DISCRETO con PWR SAVE ACTIVADO
	if ( ( MODO_DISCRETO ) && ( systemVars.pwrSave.modo == modoPWRSAVE_ON )) {

		RTC_read(&rtcDateTime);
		now = rtcDateTime.hour * 60 + rtcDateTime.min;
		pwr_save_start = systemVars.pwrSave.hora_start.hour * 60 + systemVars.pwrSave.hora_start.min;
		pwr_save_end = systemVars.pwrSave.hora_fin.hour * 60 + systemVars.pwrSave.hora_fin.min;

		if ( pwr_save_start < pwr_save_end ) {
			// Caso A:
			if ( now <= pwr_save_start ) { insidePwrSave_flag = false; goto EXIT; }
			// Caso B:
			if ( ( pwr_save_start <= now ) && ( now <= pwr_save_end )) { insidePwrSave_flag = true; goto EXIT; }
			// Caso C:
			if ( now > pwr_save_end ) { insidePwrSave_flag = false; goto EXIT; }
		}

		if (  pwr_save_end < pwr_save_start ) {
			// Caso A:
			if ( now <=  pwr_save_end ) { insidePwrSave_flag = true; goto EXIT; }
			// Caso B:
			if ( ( pwr_save_end <= now ) && ( now <= pwr_save_start )) { insidePwrSave_flag = false; goto EXIT; }
			// Caso C:
			if ( now > pwr_save_start ) { insidePwrSave_flag = true; goto EXIT; }
		}

	}

EXIT:

	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		if ( insidePwrSave_flag == true ) {
			snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::wait: inside pwrsave\r\n\0"), u_now() );
		} else {
			snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::wait: out pwrsave\r\n\0"), u_now() );
		}
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	return(insidePwrSave_flag);

}
//------------------------------------------------------------------------------------
static bool pv_procesar_signals_espera( bool *exit_flag )
{

bool ret_f = false;

	if ( GPRS_stateVars.signal_reload) {
		// Salgo a reiniciar tomando los nuevos parametros.
		*exit_flag = bool_RESTART;
		ret_f = true;
		goto EXIT;
	}

	if ( GPRS_stateVars.signal_tilt) {
		// Salgo a discar inmediatamente.
		*exit_flag = bool_CONTINUAR;
		ret_f = true;
		goto EXIT;
	}

	if ( GPRS_stateVars.signal_redial) {
		// Salgo a discar inmediatamente.
		*exit_flag = bool_CONTINUAR;
		ret_f = true;
		goto EXIT;
	}

	if ( GPRS_stateVars.signal_frameReady) {
		// Salgo a discar solo en continuo.
		if ( ! MODO_DISCRETO ) {
			*exit_flag = bool_CONTINUAR;
			ret_f = true;
			goto EXIT;
		}
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
// FUNCIONES PUBLICAS
//----------------------------------------------------------------------------------------
int32_t u_readTimeToNextDial(void)
{
	return(waiting_time);
}
//----------------------------------------------------------------------------------------
