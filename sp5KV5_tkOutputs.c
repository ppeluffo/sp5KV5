/*
 * sp5KV5_tkOutputs.c
 *
 *  Created on: 24 de jun. de 2017
 *      Author: pablo
 */

#include <sp5KV5.h>

static char out_printfBuff[CHAR64];	// Buffer de impresion

typedef enum { outESPERAR, outCHEQUEAR } t_outStates;
uint16_t OUT_timer;

static void pv_out_chequear(void);
static void pv_check_consignas(void);
static void pv_check_outputs_normales(void);
static void pv_init_outputs(void);
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
				pv_out_chequear();										// Reflejo los cambios de configuracion
			}
		}

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

RtcTimeType_t rtcDateTime;

	RTC_read(&rtcDateTime);

	if ( ( rtcDateTime.hour == systemVars.outputs.consigna_diurna.hour ) &&
			( rtcDateTime.min == systemVars.outputs.consigna_diurna.min )  ) {

		 OUT_aplicar_consigna_diurna();
		 return;
	 }

	if ( ( rtcDateTime.hour == systemVars.outputs.consigna_nocturna.hour ) &&
			( rtcDateTime.min == systemVars.outputs.consigna_nocturna.min )  ) {

		OUT_aplicar_consigna_nocturna();
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_check_outputs_normales(void)
{

	// Aplica el valor indicado en systemVars a las salidas.

	( systemVars.outputs.out0 == 0 ) ?	OUT0_off() : OUT0_on();
	( systemVars.outputs.out1 == 0 ) ?	OUT1_off() : OUT1_on();

}
//------------------------------------------------------------------------------------
static void pv_init_outputs(void)
{

	switch(systemVars.outputs.modo) {
	case OUT_OFF:
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
static void pv_init_consignas(void)
{
	// Determino cual consigna corresponde aplicar y la aplico.

RtcTimeType_t rtcDateTime;
uint16_t now, horaConsNoc, horaConsDia ;

	// Loop:
	// Hora actual en minutos.
	RTC_read(&rtcDateTime);

	// Caso 1: C.Diurna < C.Nocturna
	//           C.diurna                      C.nocturna
	// |----------|-------------------------------|---------------|
	// 0         hhmm1                          hhmm2            24
	//   nocturna             diurna                 nocturna

	systemVars.outputs.consigna_aplicada = CONSIGNA_OFF;
	horaConsDia = systemVars.outputs.consigna_diurna.hour * 60 + systemVars.outputs.consigna_diurna.min;
	horaConsNoc = systemVars.outputs.consigna_nocturna.hour * 60 + systemVars.outputs.consigna_nocturna.min;

	if ( horaConsDia < horaConsNoc ) {
		// Caso A:
		if ( now <= horaConsDia ) {
			systemVars.outputs.consigna_aplicada = CONSIGNA_NOCTURNA;
		}
		// Caso B:
		if ( ( horaConsDia <= now ) && ( now <= horaConsNoc )) {
			systemVars.outputs.consigna_aplicada = CONSIGNA_DIURNA;
		}

		// Caso C:
		if ( now > horaConsNoc ) {
			systemVars.outputs.consigna_aplicada = CONSIGNA_NOCTURNA;
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
			systemVars.outputs.consigna_aplicada = CONSIGNA_DIURNA;
		}
		// Caso B:
		if ( ( horaConsNoc <= now ) && ( now <= horaConsDia )) {
			systemVars.outputs.consigna_aplicada = CONSIGNA_NOCTURNA;
		}
		// Caso C:
		if ( now > horaConsDia ) {
			systemVars.outputs.consigna_aplicada = CONSIGNA_DIURNA;
		}
	}

	// Aplico la consigna
	switch (systemVars.outputs.consigna_aplicada) {
	case CONSIGNA_OFF:
		snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("ERROR al setear consignas: horas incompatibles\r\n\0"));
		break;
	case CONSIGNA_DIURNA:
		OUT_aplicar_consigna_diurna();
		snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("%s Consigna Inicial: Diurna\r\n\0"), u_now() );
		break;
	case CONSIGNA_NOCTURNA:
		OUT_aplicar_consigna_nocturna();
		snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("%s Consigna Inicial: Nocturna\r\n\0"), u_now() );
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

	( systemVars.outputs.out0 == 0 ) ?	OUT0_off() : OUT0_on();
	( systemVars.outputs.out1 == 0 ) ?	OUT1_off() : OUT1_on();

}
//------------------------------------------------------------------------------------
