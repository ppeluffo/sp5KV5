/*
 * sp5KV5_tkOutputs.c
 *
 *  Created on: 24 de jun. de 2017
 *      Author: pablo
 */

#include <sp5KV5.h>

static char out_printfBuff[CHAR128];	// Buffer de impresion

typedef enum { outESPERAR, outCHEQUEAR } t_outStates;
uint16_t OUT_timer;

static void pv_out_chequear(void);
static void pv_check_consignas(void);
static void pv_check_outputs_normales(void);
static void pv_init_outputs(void);
static void pv_init_outputs_off(void);
static void pv_init_consignas(void);
static void pv_init_outputs_normales(void);

//------------------------------------------------------------------------------------
void tkOutputs(void * pvParameters)
{

( void ) pvParameters;
BaseType_t xResult;
uint32_t ulNotifiedValue;
uint8_t out_state;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("starting tkOutputs..\r\n\0"));
	FreeRTOS_write( &pdUART1, out_printfBuff, sizeof(out_printfBuff) );
	out_state = outESPERAR;			// El primer estado al que voy a ir.
	OUT_timer = 10;

	pv_init_outputs();

	for( ;; )
	{

		u_kick_Wdg(WDG_OUT);

		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 1000 / portTICK_RATE_MS ) );
		// Veo si llego un mensaje
		if ( xResult == pdTRUE ) {

			if ( ( ulNotifiedValue & TK_PARAM_RELOAD ) != 0 ) {			// Mensaje de reload configuration.
				pv_init_outputs();										// Reflejo los cambios de configuracion
			}

			if ( ( ulNotifiedValue & TK_CHANGE_OUTPUTS ) != 0 ) {
				pv_check_outputs_normales();
			}

		}

		// La FSM se ejecuta solo si estoy en modo normal. En otros modos no
		// para que por ej. en modo service pueda probar las salidas.

		if ( systemVars.wrkMode == WK_NORMAL ) {

			// Recorro la maquina de estados
			switch(out_state) {
			case outCHEQUEAR:
				pv_out_chequear();
				out_state = outESPERAR;	// next state
				break;
			case outESPERAR:
				// Expiro el timer: salida normal
				OUT_timer--;
				if ( OUT_timer == 0 ) {
					OUT_timer = 25; 	// Para asegurarme chequear 2 veces por minuto
					out_state = outCHEQUEAR;
				}
				break;
			default:
				// No deberia ocurrir
				OUT_timer = 5;
				out_state = outESPERAR;
				break;
			}
		}
	}
}
//------------------------------------------------------------------------------------
static void pv_out_chequear(void)
{
	//
	switch(systemVars.outputs.modo) {
	case OUT_OFF:
		break;
	case OUT_CONSIGNA:
		pv_check_consignas();
		break;
	case OUT_NORMAL:
		pv_check_outputs_normales();
		break;
	}
}
//------------------------------------------------------------------------------------
static void pv_check_consignas(void)
{

	// Las consignas se chequean y/o setean en cualquier modo de trabajo, continuo o discreto

RtcTimeType_t rtcDateTime;

	RTC_read(&rtcDateTime);

	if ( ( rtcDateTime.hour == systemVars.outputs.consigna_diurna.hour ) &&
			( rtcDateTime.min == systemVars.outputs.consigna_diurna.min )  ) {

		OUT_aplicar_consigna_diurna();
		systemVars.outputs.consigna_aplicada = CONSIGNA_DIURNA;
		if ( (systemVars.debugLevel & (D_BASIC + D_OUTPUTS) ) != 0) {
			snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("%s OUTPUTS::check: Consigna Diurna\r\n\0"), u_now() );
			FreeRTOS_write( &pdUART1, out_printfBuff, sizeof(out_printfBuff) );
		}
		return;
	 }

	if ( ( rtcDateTime.hour == systemVars.outputs.consigna_nocturna.hour ) &&
			( rtcDateTime.min == systemVars.outputs.consigna_nocturna.min )  ) {

		OUT_aplicar_consigna_nocturna();
		systemVars.outputs.consigna_aplicada = CONSIGNA_NOCTURNA;
		if ( (systemVars.debugLevel & (D_BASIC + D_OUTPUTS) ) != 0) {
			snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("%s OUTPUTS::check: Consigna Nocturna\r\n\0"), u_now() );
			FreeRTOS_write( &pdUART1, out_printfBuff, sizeof(out_printfBuff) );
		}
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_check_outputs_normales(void)
{

	// Aplica el valor indicado en systemVars a las salidas. Solo en modo CONTINUO, ya que en modo
	// discreto consumiria mucha corriente.
	// En este modo, deberia estar configuradas las OUTPUT a OFF o CONSIGNAS

	if ( systemVars.pwrMode == PWR_DISCRETO ) {
		// No deberia entrar nunca aca pero si hay algo mal configurado ...
		//if ( (systemVars.debugLevel & D_BASIC ) != 0) {
		//	snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("%s OUTPUTS::config Error: En pwrMode discreto no pueden operar Outputs normales !!!\r\n\0"), u_now() );
		//	FreeRTOS_write( &pdUART1, out_printfBuff, sizeof(out_printfBuff) );
		//}
		return;
	}

	( systemVars.outputs.out0 == 0 ) ?	OUT0_off() : OUT0_on();
	( systemVars.outputs.out1 == 0 ) ?	OUT1_off() : OUT1_on();

}
//------------------------------------------------------------------------------------
static void pv_init_outputs(void)
{

	switch(systemVars.outputs.modo) {
	case OUT_OFF:
		pv_init_outputs_off();
		break;
	case OUT_CONSIGNA:
		pv_init_consignas();
		break;
	case OUT_NORMAL:
		pv_init_outputs_normales();
		break;
	}

}
//------------------------------------------------------------------------------------
static void pv_init_outputs_off(void)
{
	// Habilitamos al driver
	OUTPUT_DRV_enable();
	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	// Para que acepte las entradas de deshabilitar los bridges
	OUT0_disable();
	OUT1_disable();

	// Y lo dejamos durmiendo.
	OUTPUT_DRV_disable();

}
//------------------------------------------------------------------------------------
static void pv_init_consignas(void)
{
	// Determino cual consigna corresponde aplicar y la aplico.

RtcTimeType_t rtcDateTime;
uint16_t now, horaConsNoc, horaConsDia ;
uint8_t consigna_a_aplicar = 99;

	// Loop:
	// Hora actual en minutos.
	RTC_read(&rtcDateTime);

	// Caso 1: C.Diurna < C.Nocturna
	//           C.diurna                      C.nocturna
	// |----------|-------------------------------|---------------|
	// 0         hhmm1                          hhmm2            24
	//   nocturna             diurna                 nocturna

	now = rtcDateTime.hour * 60 + rtcDateTime.min;
	horaConsDia = systemVars.outputs.consigna_diurna.hour * 60 + systemVars.outputs.consigna_diurna.min;
	horaConsNoc = systemVars.outputs.consigna_nocturna.hour * 60 + systemVars.outputs.consigna_nocturna.min;

	if ( horaConsDia < horaConsNoc ) {
		// Caso A:
		if ( now <= horaConsDia ) {
			consigna_a_aplicar = CONSIGNA_NOCTURNA;
		}
		// Caso B:
		if ( ( horaConsDia <= now ) && ( now <= horaConsNoc )) {
			consigna_a_aplicar = CONSIGNA_DIURNA;
		}

		// Caso C:
		if ( now > horaConsNoc ) {
			consigna_a_aplicar = CONSIGNA_NOCTURNA;
		}
	}

	// Caso 2: C.Nocturna < Diurna
	//           C.Nocturna                      C.diurna
	// |----------|-------------------------------|---------------|
	// 0         hhmm2                          hhmm1            24
	//   diurna             nocturna                 diurna

	if (  horaConsNoc < horaConsDia ) {
		// Caso A:
		if ( now <= horaConsNoc ) {
			consigna_a_aplicar = CONSIGNA_DIURNA;
		}
		// Caso B:
		if ( ( horaConsNoc <= now ) && ( now <= horaConsDia )) {
			consigna_a_aplicar = CONSIGNA_NOCTURNA;
		}
		// Caso C:
		if ( now > horaConsDia ) {
			consigna_a_aplicar = CONSIGNA_DIURNA;
		}
	}

	// Aplico la consigna
	switch (consigna_a_aplicar) {
	case 99:
		// Incompatibilidad: seteo por default.
		snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("%s OUTPUTS::init: ERROR al setear consignas: horas incompatibles\r\n\0"), u_now());
		systemVars.outputs.modo = OUT_CONSIGNA;
		systemVars.outputs.consigna_diurna.hour = 05;
		systemVars.outputs.consigna_diurna.min = 30;
		systemVars.outputs.consigna_nocturna.hour = 23;
		systemVars.outputs.consigna_nocturna.min = 30;
		break;
	case CONSIGNA_DIURNA:
		OUT_aplicar_consigna_diurna();
		systemVars.outputs.consigna_aplicada = CONSIGNA_DIURNA;
		snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("%s OUTPUTS::init: Consigna Diurna\r\n\0"), u_now() );
		break;
	case CONSIGNA_NOCTURNA:
		OUT_aplicar_consigna_nocturna();
		systemVars.outputs.consigna_aplicada = CONSIGNA_NOCTURNA;
		snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("%s OUTPUTS::init: Consigna Nocturna\r\n\0"), u_now() );
		break;
	}

	// Exit:
	if ( (systemVars.debugLevel & (D_BASIC + D_OUTPUTS) ) != 0) {
		FreeRTOS_write( &pdUART1, out_printfBuff, sizeof(out_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
static void pv_init_outputs_normales(void)
{
	// Aplica el valor indicado en systemVars a las salidas.

	// Las salidas en modo normal NO se trabajan en pwrMode discreto.
	if ( systemVars.pwrMode == PWR_DISCRETO ) {

		OUTPUT_DRV_disable();

		if ( (systemVars.debugLevel & D_BASIC ) != 0) {
			snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("%s OUTPUTS::init: En pwrMode discreto no pueden operar Outputs normales !!!\r\n\0"), u_now() );
			FreeRTOS_write( &pdUART1, out_printfBuff, sizeof(out_printfBuff) );
		}
		return;
	}

	// En otros pwrMode dejamos habilitado el DRV.
	// Habilitamos al driver
	OUTPUT_DRV_enable();

	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	( systemVars.outputs.out0 == 0 ) ?	OUT0_off() : OUT0_on();
	( systemVars.outputs.out1 == 0 ) ?	OUT1_off() : OUT1_on();

	if ( (systemVars.debugLevel & (D_BASIC + D_OUTPUTS) ) != 0) {
		snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("%s OUTPUTS::init: Outputs normales O0=%d,O1=%d\r\n\0"), u_now(),systemVars.outputs.out0,systemVars.outputs.out1 );
		FreeRTOS_write( &pdUART1, out_printfBuff, sizeof(out_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
