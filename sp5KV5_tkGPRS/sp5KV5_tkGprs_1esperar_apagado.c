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

//------------------------------------------------------------------------------------
bool gprs_esperar_apagado(void)
{
	// Calculo el tiempo a esperar y espero. El intervalo no va a considerar el tiempo
	// posterior de proceso.

bool exit_flag = false;

// Entry:

	GPRS_stateVars.state = G_ESPERA_APAGADO;
	pub_uarts_ctl(MODEM_APAGAR);

	// Secuencia para apagar el modem y dejarlo en modo low power.
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: Apago modem\r\n\0"));
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

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

	// Al calcular el tiempo de espera ya considero si caigo dentro del periodo
	// de pwrSave. Con esto fijo el wdt y le doy 5 minutos mas.
	pv_calcular_tiempo_espera();

	while ( ! exit_flag )  {

		// Reinicio el watchdog
		pub_control_watchdog_kick(WDG_GPRS, ( waiting_time + 300));

		// Espera
		while( --waiting_time > 0 ) {
			// Espero de a 1s.
			vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

			// Proceso las señales
			if ( pv_procesar_signals_espera( &exit_flag )) {
				// Si recibi alguna senal, debo salir.
				goto EXIT;
			}
		}

		// Expiro el tiempo de espera. Veo si estoy dentro del intervalo de
		// pwrSave y entonces espero de a 10 minutos mas.
		if ( pv_check_inside_pwrSave() ) {
			waiting_time = 600;
		} else {
			// Salgo con true.
			exit_flag = bool_CONTINUAR;
			goto EXIT;
		}

	}

EXIT:

	// No espero mas y salgo del estado prender.
	waiting_time = -1;
	return(exit_flag);

}
//------------------------------------------------------------------------------------
static void pv_calcular_tiempo_espera(void)
{

static bool starting_flag = true;

	// Cuando arranco ( la primera vez) solo espero 10s y disco por primera vez
	if ( starting_flag ) {
		waiting_time = 10;
		starting_flag = false;
		goto EXIT;
	}

	// En modo MONITOR_SQE espero solo 60s
	if ( GPRS_stateVars.monitor_sqe ) {
		waiting_time = 60;
		goto EXIT;
	}

	// En modo DISCRETO ( timerDial > 900 )
	if ( MODO_DISCRETO ) {
		waiting_time = systemVars.timerDial;
		goto EXIT;
	} else {
		// En modo CONTINUO ( timerDial = 0 ) espero solo 60s.
		waiting_time = 60;
		goto EXIT;
	}

	// En cualquier otro caso no considerado, espero 60s
	waiting_time = 60;
	goto EXIT;

EXIT:

	if ( systemVars.debugLevel == D_GPRS ) {
		FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: await %lu s\r\n\0"), waiting_time );
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

bool insidePwrSave_flag = false;

#ifdef SP5KV5_3CH

RtcTimeType_t rtcDateTime;
uint16_t now, pwr_save_start, pwr_save_end ;

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

	if ( systemVars.debugLevel == D_GPRS) {
		if ( insidePwrSave_flag == true ) {
			snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: inside pwrsave\r\n\0"));
		} else {
			snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: out pwrsave\r\n\0"));
		}
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

#endif

	return(insidePwrSave_flag);

}
//-----------------------------------------------------------------------------------
static bool pv_procesar_signals_espera( bool *exit_flag )
{

bool ret_f = false;

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

	GPRS_stateVars.signal_redial = false;
	GPRS_stateVars.signal_frameReady = false;

	return(ret_f);
}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//----------------------------------------------------------------------------------------
int32_t pub_gprs_readTimeToNextDial(void)
{
	return(waiting_time);
}
//----------------------------------------------------------------------------------------
