/*
 * file_sp5K.h
 *
 *  Created on: 31/10/2015
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#ifndef SRC_SP5KLIBS_L_FILE_H_
#define SRC_SP5KLIBS_L_FILE_H_

#include "avrlibdefs.h"
#include <l_ee.h>

#include "FreeRTOS.h"
#include "FRTOS-IO.h"

//#define ff_DEBUG

#ifdef ff_DEBUG
	#define DEBUG_FF 3
#else
	#define DEBUG_FF 0
#endif


#define FF_SIZE_IN_KB	32	// Tamanio en KB de la eeprom externa.
#define FF_RECD_SIZE	64	// Tamanio del registro
#define FF_ADDR_START	0	// Posicion inicial
#define FF_MAX_RCDS		512	// Cantidad de registros
//#define FF_MAX_RCDS		128

#define FF_WRTAG	0xC5	// 1100 0101

typedef struct {		// Estructura de control de archivo
	uint16_t HEAD;		// Puntero a la primera posicion libre.
	uint16_t TAIL;		// Puntero a la primera posicion ocupada
	uint16_t RD;		// Puntero de lectura. Se mueve entre la posicion ocupada y la libre
	uint16_t rcdsFree;	// Registros libres para escribir.
	uint16_t rcds4del;	// rcds. para borrar ( espacio ocupado y leido )
	uint8_t errno;
} StatBuffer_t;

typedef struct {					// File Control Block
	StatBuffer_t ff_stat;			// Estructura de control de archivo
	char ff_buffer[FF_RECD_SIZE];	//
	char check_buffer[FF_RECD_SIZE];
} FCB_t;

FCB_t FCB;

#define pdFF_ERRNO_NONE		0
#define pdFF_ERRNO_MEMFULL	1
#define pdFF_ERRNO_MEMWR	2
#define pdFF_ERRNO_MEMEMPTY	3
#define pdFF_ERRNO_MEMRD	4
#define pdFF_ERRNO_RDCKS	5
#define pdFF_ERRNO_RDNOTAG	6
#define pdFF_ERRNO_INIT		7


//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
uint16_t FF_fopen(void);
uint8_t FF_fwrite( const void *pvBuffer, size_t xSize);
uint8_t FF_fread( void *pvBuffer, size_t xSize);
void FF_stat( StatBuffer_t *pxStatBuffer );
void FF_rewind(void);
bool FF_seek(void);
uint8_t FF_errno( void );
bool FF_del(void);
//------------------------------------------------------------------------------------

#endif /* SRC_SP5KLIBS_L_FILE_H_ */
