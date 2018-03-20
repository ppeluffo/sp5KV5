/*
 * sp5KV5_tkGprs.h
 *
 *  Created on: 26 de abr. de 2017
 *      Author: pablo
 */

#ifndef SP5KV5_TKGPRS_SP5KV5_TKGPRS_H_
#define SP5KV5_TKGPRS_SP5KV5_TKGPRS_H_

#include "../sp5KV5.h"

char gprs_printfBuff[CHAR256];

#define MAX_HW_TRIES_PWRON 		3	// Intentos de prender HW el modem
#define MAX_SW_TRIES_PWRON 		3	// Intentos de prender SW el modem
#define MAX_TRYES_NET_ATTCH		6	// Intentos de atachearme a la red GPRS
#define MAX_IP_QUERIES 			4	// Intentos de conseguir una IP
#define MAX_INIT_TRYES			4	// Intentos de procesar un frame de INIT
#define MAX_TRYES_OPEN_SOCKET	4 	// Intentos de abrir un socket
#define MAX_RCDS_WINDOW_SIZE	10	// Maximos registros enviados en un bulk de datos
#define MAX_TX_WINDOW_TRYES		4	// Intentos de enviar el mismo paquete de datos

#define G_CLR_TX	1
#define G_CLR_RX	2
#define G_CLR_BUFF	4
#define G_SHOW_RSP	8

#define IMEIBUFFSIZE	24
char buff_gprs_imei[IMEIBUFFSIZE];

struct {
	char buffer[UART0_RXBUFFER_LEN];
	u16 ptr;
} gprsRx;

typedef enum { G_ESPERA_APAGADO = 0, G_PRENDER, G_CONFIGURAR, G_MON_SQE, G_GET_IP, G_INIT_FRAME, G_DATA } t_gprs_states;

struct {
	bool modem_prendido;
	bool signal_reload;
	bool signal_redial;
	bool signal_tilt;
	bool signal_frameReady;
	uint8_t state;

} GPRS_stateVars;

#define bool_CONTINUAR	true
#define bool_RESTART	false

//bool gprs_esperar_apagado(void);
bool gprs_esperar_apagado(void);
bool gprs_prender(void);
bool gprs_configurar(void);
bool gprs_monitor_sqe(void);
bool gprs_get_ip(void);
bool gprs_init_frame(void);
bool gprs_data(void);

void g_flushRXBuffer(void);
void g_printRxBuffer(void);
void g_sleep(uint16_t timeout);
char *g_getImei(void);

bool g_open_socket(void);
bool g_socket_is_open(void);
void g_print_debug_gprs_header( const char *msg);
bool  g_procesar_signals( bool *exit_flag );

#endif /* SP5KV5_TKGPRS_SP5KV5_TKGPRS_H_ */
