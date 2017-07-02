/*
 * sp5K.h
 *
 * Created on: 27/12/2013
 *      Author: root
 */

#ifndef SP5K_H_
#define SP5K_H_

#include <avr/io.h>			/* include I/O definitions (port names, pin names, etc) */
//#include <avr/signal.h>		/* include "signal" names (interrupt names) */
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <string.h>
#include <compat/deprecated.h>
#include <util/twi.h>
#include <util/delay.h>
#include <ctype.h>
#include <util/delay.h>
#include <avr/cpufunc.h>

#include <l_adc7828.h>
#include <l_file.h>
#include <l_rtc.h>
#include <l_iopines.h>
#include <l_mcp.h>
#include <l_outputs.h>

#include "sp5Klibs/avrlibdefs.h"
#include "sp5Klibs/avrlibtypes.h"
#include "sp5Klibs/global.h"			// include our global settings
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"

#include "FRTOS-IO.h"

#include "cmdline.h"

// DEFINICION DEL TIPO DE SISTEMA
//----------------------------------------------------------------------------
#define SP5K_REV "5.0.1"
#define SP5K_DATE "@ 20170701"

#define SP5K_MODELO "sp5KV3 HW:avr1284P R5.0"
#define SP5K_VERSION "FW:FRTOS8"

#define NRO_ANALOG_CHANNELS	3
#define NRO_DIGITAL_CHANNELS 2

#define CHAR64		64
#define CHAR128	 	128
#define CHAR256	 	256

//----------------------------------------------------------------------------
// TASKS
/* Stack de las tareas */
#define tkCmd_STACK_SIZE		512
#define tkControl_STACK_SIZE	512
#define tkDigitalIn_STACK_SIZE	512
#define tkAIn_STACK_SIZE		512
#define tkGprs_STACK_SIZE		512
#define tkGprsRx_STACK_SIZE		512
#define tkOutputs_STACK_SIZE	512

/* Prioridades de las tareas */
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkControl_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkDigitalIn_TASK_PRIORITY 	( tskIDLE_PRIORITY + 1 )
#define tkAIn_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkGprs_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkGprsRx_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkOutputs_TASK_PRIORITY 	( tskIDLE_PRIORITY + 1 )

/* Prototipos de tareas */
void tkCmd(void * pvParameters);
void tkControl(void * pvParameters);
void tkDigitalIn(void * pvParameters);
void tkAnalogIn(void * pvParameters);
void tkAnalogInit(void);
void tkGprsTx(void * pvParameters);
//void tkGprsInit(void);
void tkOutputs(void * pvParameters);
void tkGprsRx(void * pvParameters);

TaskHandle_t xHandle_tkCmd, xHandle_tkControl, xHandle_tkDigitalIn, xHandle_tkAIn, xHandle_tkGprs, xHandle_tkGprsRx, xHandle_tkOutputs;

bool startTask;
typedef struct {
	uint8_t resetCause;
	uint8_t mcusr;
} wdgStatus_t;

wdgStatus_t wdgStatus;

// Mensajes entre tareas
#define TK_PARAM_RELOAD			0x01	// param reload
#define TK_READ_FRAME			0x02	// to tkAnalogIN: (mode service) read a frame
#define TK_TILT					0x04	//
#define TK_FRAME_READY			0x08	//
#define TK_REDIAL				0x10	//
//------------------------------------------------------------------------------------

xSemaphoreHandle sem_SYSVars;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

typedef enum { WK_IDLE = 0, WK_NORMAL, WK_SERVICE, WK_MONITOR_FRAME, WK_MONITOR_SQE  } t_wrkMode;
typedef enum { PWR_CONTINUO = 0, PWR_DISCRETO } t_pwrMode;
typedef enum { modoPWRSAVE_OFF = 0, modoPWRSAVE_ON } t_pwrSave;
typedef enum { D_NONE = 0, D_BASIC = 1, D_DATA = 2, D_GPRS = 4, D_MEM = 8, D_DIGITAL = 16, D_OUTPUTS = 32, D_DEBUG = 64 } t_debug;
typedef enum { T_APAGADA = 0, T_PRENDIDA = 1 } t_terminalStatus;
typedef enum { OUT_OFF = 0, OUT_CONSIGNA, OUT_NORMAL } t_outputs;
typedef enum { CONSIGNA_DIURNA = 0, CONSIGNA_NOCTURNA } t_consigna_aplicada;

#define NRO_CHANNELS		3

#define DLGID_LENGTH		12
#define APN_LENGTH			32
#define PORT_LENGTH			7
#define IP_LENGTH			24
#define SCRIPT_LENGTH		64
#define PASSWD_LENGTH		15
#define PARAMNAME_LENGTH	5

typedef struct {
	uint8_t hour;
	uint8_t min;
} time_t;

typedef struct {
	uint8_t level[2];				// 2
	double pulses[2];				// 8
} dinData_t;		// 14 bytes

typedef struct {
	// size = 7+5+5+4+3*4+1 = 33 bytes
	RtcTimeType_t rtc;				// 7
	dinData_t dIn;					// 12
	double analogIn[NRO_CHANNELS];	// 12
	double batt;					// 4

} frameData_t;	// 38 bytes

typedef struct {
	uint8_t modo;
	uint8_t out0;
	uint8_t out1;
	time_t consigna_diurna;
	time_t consigna_nocturna;
	uint8_t consigna_aplicada;
} outputs_t;

typedef struct {
	uint8_t modo;
	time_t hora_start;
	time_t hora_fin;
} pwrsave_t;

typedef struct {
	// Variables de trabajo.
	// Tamanio: 302 bytes para 3 canales.

	uint8_t dummyBytes;
	uint8_t initByte;

	char dlgId[DLGID_LENGTH];
	char apn[APN_LENGTH];
	char server_tcp_port[PORT_LENGTH];
	char server_ip_address[IP_LENGTH];
	char dlg_ip_address[IP_LENGTH];
	char serverScript[SCRIPT_LENGTH];
	char passwd[PASSWD_LENGTH];

	uint8_t csq;
	uint8_t dbm;
	uint8_t ri;
	uint8_t termsw;
	bool terminal_on;

	uint16_t timerPoll;
	uint32_t timerDial;

	t_wrkMode wrkMode;
	t_pwrMode pwrMode;

	uint8_t logLevel;		// Nivel de info que presentamos en display.
	uint8_t debugLevel;		// Indica que funciones debugear.
	uint8_t gsmBand;

	pwrsave_t pwrSave;

	// Nombre de los canales
	char aChName[NRO_CHANNELS][PARAMNAME_LENGTH];
	char dChName[2][PARAMNAME_LENGTH];

	// Configuracion de Canales analogicos
	uint8_t Imin[NRO_CHANNELS];				// Coeficientes de conversion de I->magnitud (presion)
	uint8_t Imax[NRO_CHANNELS];
	uint8_t Mmin[NRO_CHANNELS];
	double Mmax[NRO_CHANNELS];

	// Configuracion de canales digitales
	double magPP[2];

	bool roaming;

	bool tiltEnabled;

	outputs_t outputs;

} systemVarsType;	// 315 bytes

systemVarsType systemVars,tmpSV;

#define EEADDR_SV 32		// Direccion inicio de la EE de escritura del systemVars.

//------------------------------------------------------------------------------------
// FUNCIONES DE USO GENERAL.
//------------------------------------------------------------------------------------
void u_readDigitalCounters( dinData_t *dIn , bool resetCounters );
void u_panic( uint8_t panicCode );
bool u_configOutputs( uint8_t modo, char *param1, char *param2 );
bool u_configAnalogCh( uint8_t channel, char *chName, char *s_iMin, char *s_iMax, char *s_mMin, char *s_mMax );
bool u_configDigitalCh( uint8_t channel, char *chName, char *s_magPP );
bool u_configPwrMode(uint8_t pwrMode);
bool u_configTimerPoll(char *s_tPoll);
bool u_configTimerDial(char *s_tDial);
void u_configPwrSave(uint8_t modoPwrSave, char *s_startTime, char *s_endTime);
void u_kick_Wdg( uint8_t wdgId );
bool u_saveSystemParams(void);
bool u_loadSystemParams(void);
void u_loadDefaults(void);
char *u_now(void);
void u_debugPrint(uint8_t debugCode, char *msg, uint16_t size);
void u_readDataFrame (frameData_t *dFrame);
int16_t u_readTimeToNextPoll(void);
void u_reset(void);
bool u_terminal_is_on(void);
bool u_tilt_alarmFired(void);
int32_t u_readTimeToNextDial(void);

char nowStr[32];
char debug_printfBuff[CHAR128];

//------------------------------------------------------------------------------------
// PANIC CODES
#define P_AIN_TIMERSTART	1
#define P_AIN_TIMERCREATE	2

//------------------------------------------------------------------------------------
// WATCHDOG
uint8_t systemWdg;

#define WDG_CTL			0x01
#define WDG_CMD			0x02
#define WDG_DIN			0x04
#define WDG_OUT			0x08
#define WDG_AIN			0x10
//#define WDG_GPRS		0x20
//#define WDG_GPRSRX	0x40

//------------------------------------------------------------------------------------

#endif /* SP5K_H_ */
