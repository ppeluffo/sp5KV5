/*
 * sp5KV5_tkOutputs.c
 *
 *  Created on: 24 de jun. de 2017
 *      Author: pablo
 */


#include <sp5KV5.h>

// La tarea pasa por el mismo lugar c/25s.
#define WDG_OUT_TIMEOUT	60

#ifdef SP5KV5_3CH

static char out_printfBuff[CHAR128];	// Buffer de impresion

typedef enum { outESPERAR, outCHEQUEAR } t_outStates;

static void pv_out_chequear(void);
static void pv_out_check_consignas(void);
static void pv_out_check_outputs_normales(void);
static void pv_out_init(void);
static void pv_out_init_outputs_off(void);
static void pv_out_init_consignas(void);
static void pv_out_init_outputs_normales(void);

static 	uint8_t l_out_A, l_out_B;
static bool exit_loop_to_reconfigure;

//------------------------------------------------------------------------------------
void tkOutputs(void * pvParameters)
{
( void ) pvParameters;
uint8_t timer;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	FRTOS_snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("starting tkOutputs..\r\n\0"));
	FreeRTOS_write( &pdUART1, out_printfBuff, sizeof(out_printfBuff) );

	pv_out_init();

	for( ;; )
	{

		pub_control_watchdog_kick(WDG_OUT, WDG_OUT_TIMEOUT);

		// Espero con lo que puedo entrar en tickless
		// Con 25s aseguro chequear 2 veces por minuto.
		exit_loop_to_reconfigure = false;
		for ( timer = 0; timer < 25; timer++) {
			vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
			if ( exit_loop_to_reconfigure );
				break;
		}

		// Chequeo y aplico.
		pv_out_chequear();
	}
}
//------------------------------------------------------------------------------------
#endif /* SP5KV5_3CH */

#ifdef SP5KV5_8CH
//------------------------------------------------------------------------------------
void tkOutputs(void * pvParameters)
{
( void ) pvParameters;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );


	for( ;; )
	{

		pub_control_watchdog_kick(WDG_OUT, WDG_OUT_TIMEOUT);
		vTaskDelay( ( TickType_t)( 30000 / portTICK_RATE_MS ) );
	}
}
//------------------------------------------------------------------------------------
#endif /* SP5KV5_8CH */

#ifdef SP5KV5_3CH

static void pv_out_chequear(void)
{

	switch(systemVars.outputs.modo) {
	case OUT_OFF:
		break;
	case OUT_CONSIGNA:
		pv_out_check_consignas();
		break;
	case OUT_NORMAL:
		pv_out_check_outputs_normales();
		break;
	}
}
//------------------------------------------------------------------------------------
static void pv_out_check_consignas(void)
{
	// Las consignas se chequean y/o setean en cualquier modo de trabajo, continuo o discreto

RtcTimeType_t rtcDateTime;

	RTC_read(&rtcDateTime);

	if ( ( rtcDateTime.hour == systemVars.outputs.consigna_diurna.hour ) &&
			( rtcDateTime.min == systemVars.outputs.consigna_diurna.min )  ) {

		pub_output_set_consigna_diurna();
		return;
	 }

	if ( ( rtcDateTime.hour == systemVars.outputs.consigna_nocturna.hour ) &&
			( rtcDateTime.min == systemVars.outputs.consigna_nocturna.min )  ) {

		pub_output_set_consigna_nocturna();
		return;
	}
}
//------------------------------------------------------------------------------------
static void pv_out_check_outputs_normales(void)
{

	// Solo cambio las salidas si cambio el systemVars.
	if ( l_out_A != systemVars.outputs.out_A) {
		l_out_A = systemVars.outputs.out_A;
		( l_out_A == 0 ) ?	DRV8814_set('A',"01") :  DRV8814_set('A',"10");
	}

	if ( l_out_B != systemVars.outputs.out_B ) {
		l_out_B = systemVars.outputs.out_B;
		( l_out_B == 0 ) ?	DRV8814_set('B',"01") :  DRV8814_set('B',"10");
	}

}
//------------------------------------------------------------------------------------
static void pv_out_init(void)
{

	switch(systemVars.outputs.modo) {
	case OUT_OFF:
		pv_out_init_outputs_off();
		break;
	case OUT_CONSIGNA:
		pv_out_init_consignas();
		break;
	case OUT_NORMAL:
		pv_out_init_outputs_normales();
		break;
	}

}
//------------------------------------------------------------------------------------
static void pv_out_init_outputs_off(void)
{
	// Habilitamos al driver
	IO_set_SLP();
	IO_set_RES();

	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	IO_clr_ENA();
	IO_clr_ENB();
	IO_clr_PHA();
	IO_clr_PHB();

	// Y lo dejamos durmiendo.
	IO_clr_SLP();
}
//------------------------------------------------------------------------------------
static void pv_out_init_consignas(void)
{
	// Determino cual consigna corresponde aplicar y la aplico.

RtcTimeType_t rtcDateTime;
uint16_t now, horaConsNoc, horaConsDia ;
uint8_t consigna_a_aplicar = 99;

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
		FRTOS_snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("OUTPUTS: INIT ERROR al setear consignas: horas incompatibles\r\n\0"));
		systemVars.outputs.modo = OUT_CONSIGNA;
		systemVars.outputs.consigna_diurna.hour = 05;
		systemVars.outputs.consigna_diurna.min = 30;
		systemVars.outputs.consigna_nocturna.hour = 23;
		systemVars.outputs.consigna_nocturna.min = 30;
		break;
	case CONSIGNA_DIURNA:
		pub_output_set_consigna_diurna();
		break;
	case CONSIGNA_NOCTURNA:
		pub_output_set_consigna_nocturna();
		break;
	}

}
//------------------------------------------------------------------------------------
static void pv_out_init_outputs_normales(void)
{
	// Aplica el valor indicado en systemVars a las salidas.

	// Habilitamos al driver
	IO_set_SLP();
	IO_set_RES();

	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	l_out_A = systemVars.outputs.out_A;
	l_out_B = systemVars.outputs.out_B;

	( l_out_A == 0 ) ?	DRV8814_set('A',"01") :  DRV8814_set('A',"10");
	( l_out_B == 0 ) ?	DRV8814_set('B',"01") :  DRV8814_set('B',"10");

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void pub_outputs_load_defaults(void)
{

	systemVars.outputs.modo = OUT_OFF;
	systemVars.outputs.out_A = 0;
	systemVars.outputs.out_B = 0;
	systemVars.outputs.consigna_diurna.hour = 05;
	systemVars.outputs.consigna_diurna.min = 30;
	systemVars.outputs.consigna_nocturna.hour = 23;
	systemVars.outputs.consigna_nocturna.min = 30;

}
//------------------------------------------------------------------------------------
void pub_outputs_config( uint8_t param0, char *param1, char *param2 )
{
	// Configura las salidas en el systemVars.

//	FRTOS_snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("DEBUG: p0=[%d], p1=[%s], p2=[%s]\r\n\0"),param0, param1, param2);
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	// param0 es el modo.
	switch(param0) {
	case OUT_OFF:
		systemVars.outputs.modo = OUT_OFF;
		break;
	case OUT_CONSIGNA:
		systemVars.outputs.modo = OUT_CONSIGNA;
		if ( param1 != NULL ) { pub_convert_str_to_time_t(param1, &systemVars.outputs.consigna_diurna); }
		if ( param2 != NULL ) { pub_convert_str_to_time_t(param2, &systemVars.outputs.consigna_nocturna); }
		//pv_out_init_consignas();
		break;
	case OUT_NORMAL:
		systemVars.outputs.modo = OUT_NORMAL;
		systemVars.outputs.out_A = 0;
		systemVars.outputs.out_B = 0;
		//pv_out_init_outputs_normales();
		break;
	}

	// Obligo a salir del loop a la tarea para reconfigurarse
	exit_loop_to_reconfigure = true;
}
//----------------------------------------------------------------------------------------
void pub_output_set_consigna_diurna(void)
{
	// En consigna diurna la valvula A (JP28) queda abierta y la valvula B (JP2) cerrada.
	// Aplico un pulso + a la valvula A y un pulso - a la B.
//	DRV8814_test_pulse("A", "+","100");
	DRV8814_open_valve_A();
	vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );
//	DRV8814_test_pulse("B", "+","100");
	DRV8814_close_valve_B();

	systemVars.outputs.consigna_aplicada = CONSIGNA_DIURNA;
	FRTOS_snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("OUTPUTS: Aplico Consigna Diurna\r\n\0" ));
	FreeRTOS_write( &pdUART1, out_printfBuff, sizeof(out_printfBuff) );
}
//----------------------------------------------------------------------------------------
void pub_output_set_consigna_nocturna(void)
{

//	DRV8814_test_pulse("A","-","100");
	DRV8814_close_valve_A();
	vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );
//	DRV8814_test_pulse("B", "-","100");
	DRV8814_open_valve_B();

	systemVars.outputs.consigna_aplicada = CONSIGNA_NOCTURNA;
	FRTOS_snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("OUTPUTS: Aplico Consigna Nocturna\r\n\0" ));
	FreeRTOS_write( &pdUART1, out_printfBuff, sizeof(out_printfBuff) );
}
//----------------------------------------------------------------------------------------
void pub_output_set_outputs( char id_output, uint8_t value)
{

	switch(id_output) {
	case 'A':
		( value == 0 ) ? DRV8814_set('A',"01") :  DRV8814_set('A',"10");
		systemVars.outputs.out_A = value;
		break;
	case 'B':
		( value == 0 ) ? DRV8814_set('B',"01") :  DRV8814_set('B',"10");
		systemVars.outputs.out_B = value;
		break;
	}

	FRTOS_snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("OUTPUTS: Set out_%c=%d\r\n\0"),id_output,value );
	FreeRTOS_write( &pdUART1, out_printfBuff, sizeof(out_printfBuff) );

}
//----------------------------------------------------------------------------------------

#endif /* SP5KV5_3CH */


