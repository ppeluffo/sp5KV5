/*
 * file_sp5K.c
 *
 *  Created on: 31/10/2015
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

//------------------------------------------------------------------------------------

#include <l_file.h>

//#define DEBUG_FF

static uint8_t pv_memChecksum( uint8_t *buff, uint8_t limit );

//------------------------------------------------------------------------------------
uint16_t FF_fopen(void)
{
	/*  Debe correrse luego de iniciado el FRTOS ya que utiliza funciones de este !!!
	    Abre el archivo de memoria extEE.
	    Lo recorre buscando el ppio. y el final e inicializa el FCB
	 	Inicializa el sistema de la memoria ( punteros )
		Recorro la memoria buscando transiciones VACIO->DATO y DATO->VACIO.
		La transicion VACIO->DATO determina el puntero DELptr
		La transicion DATO->VACIO determina el puntero WRptr.
		Si no hay transiciones y todos son datos, la memoria esta llena.
		Si no hay transicions y todos son blancos, la memoria esta vacia.
		Si todo anda bien, retorna en ERRNO un NONE.
		En otro caso retorna el recdNbr del error y setea la variable ERRNO

		// Testing con buffer de 16 posiciones:
		// Memoria vacia: OK
		// Memoria llena: OK
		// Memoria con HEAD(10) > TAIL(4), Free(10) OK
		// Memoria con HEAD(3) < TAIL(8), Free(5) OK
		// Condicion de borde 1: HEAD(15), TAIL(0), Free(1) OK
		// Condicion de borde 2: HEAD(0), TAIL(1), Free(1) OK
		// Condicion de borde 3: HEAD(0), TAIL(15), Free(15) OK
		// Condicion de borde 4: HEAD(1), TAIL(0), Free(15) OK

	 */

uint8_t mark_Z, mark;
uint16_t recd_id;
bool transicion = false;
uint8_t bytes_read = 0U;
uint16_t eeAddress = 0;

#ifdef ff_DEBUG
	FCB.ff_stat.HEAD = 0;
	FCB.ff_stat.TAIL = 0;
	FCB.ff_stat.RD  = FCB.ff_stat.TAIL;
	FCB.ff_stat.rcdsFree = FF_MAX_RCDS;
	return(FF_MAX_RCDS -1);
#endif

	// Lo primero es obtener el semaforo del I2C
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_FF);

	FCB.ff_stat.errno = pdFF_ERRNO_NONE;

	// El primer registro que leo es el ultimo del archivo
	// direccion de lectura
	recd_id = (FF_MAX_RCDS -1);
	eeAddress = FF_ADDR_START +  recd_id * FF_RECD_SIZE;
	// leo una pagina entera, (recd) 64 bytes.
	memset( FCB.ff_buffer,0, sizeof(FCB.ff_buffer) );
	bytes_read = EE_read( eeAddress, &FCB.ff_buffer, FF_RECD_SIZE );

#ifdef DEBUG_FF
	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("FO: [%d][%d]\r\n\0"),FF_RECD_SIZE, bytes_read);
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif

	if ( bytes_read != FF_RECD_SIZE ) {
		FCB.ff_stat.errno = pdFF_ERRNO_INIT;
		goto quit;
	}

	mark_Z = FCB.ff_buffer[sizeof(FCB.ff_buffer) - 1];

	// Recorro toda la memoria EE buscando transiciones.
	for ( recd_id=0; recd_id < FF_MAX_RCDS; recd_id++) {

		// Para no salir por wdg reset
		if ( (recd_id % 128) == 0 ) {
			wdt_reset();
		}

		eeAddress = FF_ADDR_START + recd_id * FF_RECD_SIZE;
		// leo una pagina entera, (recd) 64 bytes.
		memset( FCB.ff_buffer,0, sizeof(FCB.ff_buffer) );
		bytes_read = EE_read(eeAddress, FCB.ff_buffer, FF_RECD_SIZE);

#ifdef DEBUG_FF
//		snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("FO: [%d][%d][%d][%d]\r\n\0"),xPos, val,FF_RECD_SIZE, xReturn);
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
		if ( bytes_read != FF_RECD_SIZE )  {
			FCB.ff_stat.errno = pdFF_ERRNO_INIT;
			goto quit;
		}

		mark = FCB.ff_buffer[sizeof(FCB.ff_buffer) - 1];

#ifdef DEBUG_FF
//		if ( mark == FF_WRTAG ) {
//			snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("FO: [%d][%d][%d][%d][0X%03x]\r\n\0"),xPos, val,FF_RECD_SIZE, xReturn, mark);
//			FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
//		}
#endif

		// busco transiciones:
		if ( ( mark_Z == 0) && ( mark == FF_WRTAG ) ) {
			// Tengo una transicion VACIO->DATO.
			FCB.ff_stat.TAIL = recd_id;
			transicion = true;
		}

		if ( ( mark_Z == FF_WRTAG ) && ( mark == 0) ) {
			// Tengo una transicion DATO->VACIO.
			FCB.ff_stat.HEAD = recd_id;
			transicion = true;
		}

		mark_Z = mark;
	}

	// Recorri toda la memoria. Analizo las transiciones...
	if ( ! transicion ) {
		// Si no hubieron transiciones es que la memoria esta llena o vacia.
		if ( mark == 0 ) {
			// Memoria vacia.
			FCB.ff_stat.HEAD = 0;
			FCB.ff_stat.TAIL = 0;
			FCB.ff_stat.RD  = FCB.ff_stat.TAIL;
			FCB.ff_stat.rcdsFree = FF_MAX_RCDS;
		} else {
			// Memoria llena
			FCB.ff_stat.HEAD = 0;
			FCB.ff_stat.TAIL = 0;
			FCB.ff_stat.RD  = FCB.ff_stat.TAIL;
			FCB.ff_stat.rcdsFree = 0;
		}
	} else {
		// Memoria con datos. Calculo los registro ocupados.
		if ( FCB.ff_stat.HEAD > FCB.ff_stat.TAIL) {
			FCB.ff_stat.RD  = FCB.ff_stat.TAIL;
			FCB.ff_stat.rcdsFree = FF_MAX_RCDS - FCB.ff_stat.HEAD + FCB.ff_stat.TAIL;
		} else {
			FCB.ff_stat.RD  = FCB.ff_stat.TAIL;
			FCB.ff_stat.rcdsFree = FCB.ff_stat.TAIL - FCB.ff_stat.HEAD;
		}
	}

quit:

	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_FF);

#ifdef DEBUG_FF
//		snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("FO: [%d][%d]\r\n\0"),xPos, FCB.ff_stat.errno);
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif

	return(recd_id);

}
//------------------------------------------------------------------------------------
uint8_t FF_fwrite( const void *pvBuffer, size_t xSize)
{
	// El archivo es del tipo circular FirstIN-LastOUT.
	// Escribe un registro en la posicion apuntada por el HEAD.
	// El registro que se pasa en pvBuffer es del tipo 'frameData_t' de 38 bytes
	// pero en la memoria voy a escribir de a paginas de 64 bytes.
	// Retorna el nro.de bytes escritos y setea la variable 'errno' del FCB
	// En la posicion 63 grabo un tag con el valor 0xC5 para poder luego
	// determinar si el registro esta ocupado o vacio.

	// TESTING:
	// Memoria vacia: OK
	// Memoria llena: OK
	// Memoria con HEAD(10) > TAIL(4), Free(10) OK
	// Memoria con HEAD(3) < TAIL(8), Free(5) OK
	// Condicion de borde 1: HEAD(15), TAIL(0), Free(1) OK
	// Condicion de borde 2: HEAD(0), TAIL(1), Free(1) OK

uint16_t tryes;
uint16_t eeAddress = 0;
uint8_t bytes_writen = 0U;

#ifdef ff_DEBUG
	snprintf_P( debug_printfBuff,CHAR128,PSTR("**DEBUG FF_WRITE: 0x%02x\r\n\0"),xSize );
	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif

	// Lo primero es obtener el semaforo del I2C
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_FF);
	FCB.ff_stat.errno = pdFF_ERRNO_NONE;

	// Si la memoria esta llena no puedo escribir: salgo
	if ( FCB.ff_stat.rcdsFree == 0 ) {
#ifdef ff_DEBUG
		snprintf_P( debug_printfBuff,CHAR128,PSTR("**DEBUG FF_WRITE: MEM_FULL\r\n\0") );
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
		FCB.ff_stat.errno = pdFF_ERRNO_MEMFULL;
		goto quit;
	}

	// inicializo la estructura lineal temporal en el FCB para copiar ahi los datos y
	// calcular el checksum antes de grabarlo en memoria.
	memset( FCB.ff_buffer,0, sizeof(FCB.ff_buffer) );
	// copio los datos recibidos del frame al buffer ( 0..(xSize-1))
	memcpy ( FCB.ff_buffer, pvBuffer, xSize );
	// Calculo y grabo el checksum a continuacion del frame (en la pos.xSize)
	// El checksum es solo del dataFrame por eso paso dicho size.
	FCB.ff_buffer[xSize] = pv_memChecksum(FCB.ff_buffer, xSize );
#ifdef ff_DEBUG
		snprintf_P( debug_printfBuff,CHAR128,PSTR("**DEBUG FF_WRITE: CKS=%d\r\n\0"),FCB.ff_buffer[xSize] );
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif

	// Grabo el tag para indicar que el registro esta escrito.
	FCB.ff_buffer[sizeof(FCB.ff_buffer) - 1] = FF_WRTAG;

	// EE WRITE:
	// Direccion interna en la EE.(comienzo del registro / frontera)
	eeAddress = FF_ADDR_START + FCB.ff_stat.HEAD * FF_RECD_SIZE;

	// Escribo la memoria. Escribo un pagina entera, 64 bytes.
	// Reintento hasta 3 veces.
	for ( tryes = 0; tryes < 3; tryes++ ) {
		// Write
		bytes_writen = EE_write(eeAddress, FCB.ff_buffer, FF_RECD_SIZE);
		taskYIELD();
		// Verify
		EE_read(eeAddress, FCB.check_buffer, FF_RECD_SIZE);

#ifdef ff_DEBUG
		snprintf_P( debug_printfBuff,CHAR128,PSTR("**DEBUG FF_WRITE: tryes:0x%02x, eeAddress:0x%02x, bytes_written:0x%02x\r\n\0"),tryes,eeAddress,bytes_writen );
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif

		if ( memcmp (&FCB.check_buffer, &FCB.ff_buffer, FF_RECD_SIZE) == 0 )
			break;

		if  ( tryes == 3 ) {
			snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("FS WR ERR\r\n\0"));
			FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff));
			FCB.ff_stat.errno = pdFF_ERRNO_MEMWR;
			bytes_writen = 0U;
			goto quit;
		}
	}

	if (bytes_writen != FF_RECD_SIZE ) {
		// Errores de escritura ?
		FCB.ff_stat.errno = pdFF_ERRNO_MEMWR;
		bytes_writen = 0U;
		goto quit;
	} else {
		bytes_writen = xSize;
		// Avanzo el puntero de WR en modo circular
		FCB.ff_stat.HEAD = (++FCB.ff_stat.HEAD == FF_MAX_RCDS) ?  0 : FCB.ff_stat.HEAD;
		FCB.ff_stat.rcdsFree--;
	}

quit:
	// libero los semaforos
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_FF);
	return(bytes_writen);

}
//------------------------------------------------------------------------------------
uint8_t FF_fread( void *pvBuffer, size_t xSize)
{
	// Lee un registro apuntado por RD.
	// Retorna la cantidad de bytes leidos.
	// Las condiciones de lectura son:
	// - la memoria debe tener al menos algun dato
	// - el puntero RD debe apuntar dentro del bloque 'leible'
	//
	// Lee un registro ( como string ) y lo copia al espacio de memoria apuntado
	// *pvBuffer. El puntero es void y entonces debemos pasar el tamaño de la estructura
	// a la que apunta.
	// En caso que tenga problemas para leer o de checksum, debo continuar e indicar
	// el error.

uint8_t rdCheckSum;
uint16_t eeAddress = 0;
uint8_t bytes_read = 0U;

#ifdef ff_DEBUG
		snprintf_P( debug_printfBuff,CHAR128,PSTR("**DEBUG FF_READ:\r\n\0") );
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif

	// Lo primero es obtener el semaforo del I2C
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_FF);

	FCB.ff_stat.errno = pdFF_ERRNO_NONE;

	// Si la memoria esta vacia salgo ( todos los registros libres )
	if ( FCB.ff_stat.rcdsFree == FF_MAX_RCDS ) {
#ifdef ff_DEBUG
		snprintf_P( debug_printfBuff,CHAR128,PSTR("**DEBUG FF_WRITE: MEM_EMPTY\r\n\0") );
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
		FCB.ff_stat.errno = pdFF_ERRNO_MEMEMPTY;
		goto quit;
	}

	// Si el registro no corresponde al bloque 'leible', salgo
	if ( ( FCB.ff_stat.HEAD > FCB.ff_stat.TAIL) && ( ( FCB.ff_stat.RD >= FCB.ff_stat.HEAD ) || ( FCB.ff_stat.RD < FCB.ff_stat.TAIL ) ) ) {
		FCB.ff_stat.errno = pdFF_ERRNO_MEMEMPTY;
		goto quit;
	}
	if ( ( FCB.ff_stat.HEAD < FCB.ff_stat.TAIL) && ( ( FCB.ff_stat.RD >= FCB.ff_stat.HEAD ) && ( FCB.ff_stat.RD < FCB.ff_stat.TAIL ) ) ) {
		FCB.ff_stat.errno = pdFF_ERRNO_MEMEMPTY;
		goto quit;
	}

	// Aqui es que estoy dentro de un bloque 'leible'

	// inicializo la estructura lineal temporal en el FCB.
	memset( FCB.ff_buffer,0, sizeof(FCB.ff_buffer) );
	// EE READ:
	// Direccion interna en la EE.(comienzo del registro / frontera)
	eeAddress = FF_ADDR_START + FCB.ff_stat.RD * FF_RECD_SIZE;
	bytes_read = EE_read(eeAddress, FCB.ff_buffer, FF_RECD_SIZE);

#ifdef ff_DEBUG
		snprintf_P( debug_printfBuff,CHAR128,PSTR("**DEBUG FF_READ: eeAddress:0x%02x, bytes_read:0x%02x\r\n\0"),eeAddress,bytes_read );
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif

	// Avanzo el puntero de RD en modo circular siempre !!
	FCB.ff_stat.RD = (++FCB.ff_stat.RD == FF_MAX_RCDS) ?  0 : FCB.ff_stat.RD;

	// Copio los datos a la estructura de salida.: aun no se si estan correctos
	memcpy( pvBuffer, &FCB.ff_buffer, xSize );

	// Errores de lectura ?
	// Solo indico los errores, pero igual devuelvo el recd. para no trancarme
	if (bytes_read != FF_RECD_SIZE ) {
		FCB.ff_stat.errno = pdFF_ERRNO_MEMRD;
		bytes_read = 0U;
		goto quit;
	}

	// Verifico los datos leidos ( checksum )
	// El checksum es solo del dataFrame por eso paso dicho size.
	rdCheckSum = pv_memChecksum(FCB.ff_buffer, xSize );
#ifdef ff_DEBUG
		snprintf_P( debug_printfBuff,CHAR128,PSTR("**DEBUG FF_READ: CKS=%d,%d\r\n\0"),rdCheckSum, FCB.ff_buffer[xSize] );
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
	if ( rdCheckSum != FCB.ff_buffer[xSize] ) {
		FCB.ff_stat.errno = pdFF_ERRNO_RDCKS;
		bytes_read = 0U;
		goto quit;
	}

	// Vemos si la ultima posicion tiene el tag de ocupado.
	if ( ( FCB.ff_buffer[sizeof(FCB.ff_buffer) - 1] )  != FF_WRTAG ) {
		FCB.ff_stat.errno = pdFF_ERRNO_RDNOTAG;
		bytes_read = 0U;
		goto quit;
	}

	// Datos leidos correctamente
	//bytes_read = xSize;

quit:
	// libero los semaforos
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_FF);

	return(bytes_read);
}
//------------------------------------------------------------------------------------
void FF_stat( StatBuffer_t *pxStatBuffer )
{
	// Debe calcularse el contador de los registros ocupados que pueden ser borrados.( ya leidos )
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_FF);

	// Caso 1: Memoria vacia:
	if ( FCB.ff_stat.rcdsFree == FF_MAX_RCDS ) {
		FCB.ff_stat.rcds4del = 0;
		goto quit;
	}

	// Caso 2: Memoria llena:
	if ( FCB.ff_stat.rcdsFree == 0 ) {
		FCB.ff_stat.rcds4del = FCB.ff_stat.RD - FCB.ff_stat.TAIL;
		goto quit;
	}

	// El puntero RD debe estar en un bloque 'leible'
	if ( ( FCB.ff_stat.HEAD > FCB.ff_stat.TAIL) && ( FCB.ff_stat.RD >= FCB.ff_stat.TAIL ) && ( FCB.ff_stat.RD <= FCB.ff_stat.HEAD ) ) {
		FCB.ff_stat.rcds4del = FCB.ff_stat.RD - FCB.ff_stat.TAIL;
		goto quit;
	}

	if ( ( FCB.ff_stat.HEAD < FCB.ff_stat.TAIL) && ( FCB.ff_stat.RD >= FCB.ff_stat.TAIL ) ) {
		FCB.ff_stat.rcds4del = FCB.ff_stat.RD - FCB.ff_stat.TAIL;
		goto quit;
	}

	if ( ( FCB.ff_stat.HEAD < FCB.ff_stat.TAIL ) && ( FCB.ff_stat.RD <= FCB.ff_stat.HEAD ) ) {
		FCB.ff_stat.rcds4del = FF_MAX_RCDS - FCB.ff_stat.TAIL + FCB.ff_stat.RD;
		goto quit;
	}

quit:
	memcpy( pxStatBuffer, &FCB.ff_stat, sizeof(StatBuffer_t));
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_FF);

}
//------------------------------------------------------------------------------------
bool FF_seek(void)
{
	// Ajusta la posicion del puntero de lectura al primer registro a leer, es decir
	// al DELptr.
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_FF);
	FCB.ff_stat.RD = FCB.ff_stat.TAIL;
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_FF);
	return(true);

}
//------------------------------------------------------------------------------------
bool FF_del(void)
{
	// Borra un registro apuntado por TAIL.

uint16_t eeAddress = 0;

	// Lo primero es obtener el semaforo del I2C
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_FF);
	FCB.ff_stat.errno = pdFF_ERRNO_NONE;

	// Si la memoria esta vacia salgo
	if ( FCB.ff_stat.rcdsFree == FF_MAX_RCDS ) {
		goto quit;
	}

	// Voy a escribir un registro en blanco
	memset( FCB.ff_buffer,0, sizeof(FCB.ff_buffer) );

	// en la direccion interna en la EE.(comienzo del registro / frontera)
	eeAddress = FF_ADDR_START + FCB.ff_stat.TAIL * FF_RECD_SIZE;

	// Escribo un pagina entera, 64 bytes.
	// Por ahora no controlo los errores de borrado
	EE_write( eeAddress, FCB.ff_buffer, FF_RECD_SIZE );

	FCB.ff_stat.rcdsFree++;
	FCB.ff_stat.TAIL = (++FCB.ff_stat.TAIL == FF_MAX_RCDS) ?  0 : FCB.ff_stat.TAIL;

quit:

	// libero los semaforos
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_FF);
	return(true);
}
//------------------------------------------------------------------------------------
void FF_rewind(void)
{
	// Borra el archivo y lo lleva a su condicion inicial.
	// Inicializa la memoria. Como lleva bastante tiempo, tenemos problemas con el
	// watchdog. Por esto desde donde la invocamos debemos desactivarlo y esta
	// funcion SOLO debe usarse desde CMD.

uint16_t tryes;
uint16_t rcd_nbr;
uint16_t eeAddress = 0;

	wdt_reset();

	// Lo primero es obtener el semaforo del I2C
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_FF);

	// inicializo la estructura lineal temporal del FCB.
	memset( FCB.ff_buffer,0, sizeof(FCB.ff_buffer) );

	// EE WRITE:
	// Ciclo de borrado
	for ( rcd_nbr = 0; rcd_nbr < FF_MAX_RCDS; rcd_nbr++) {
		// direccion interna en la EE.(comienzo del registro / frontera)
		eeAddress = FF_ADDR_START + rcd_nbr * FF_RECD_SIZE;

		for ( tryes = 0; tryes < 3; tryes++ ) {
			// Borro: escribo un pagina entera, 64 bytes con los '\0'
			EE_write(eeAddress, FCB.ff_buffer, FF_RECD_SIZE);
			taskYIELD();
			// Leo y verifico
			EE_read(eeAddress, FCB.check_buffer, FF_RECD_SIZE);
			if ( memcmp (&FCB.check_buffer, &FCB.ff_buffer, FF_RECD_SIZE) == 0 )
				break;
			if  ( tryes == 3 ) {
				snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("FFrew ERR: %d,%d\r\n\0"),rcd_nbr, eeAddress);
				FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff));
			}

		}
		// Imprimo realimentacion c/32 recs.
		if ( (rcd_nbr % 32) == 0 ) {
			FreeRTOS_write( &pdUART1, ".\0", sizeof(".\0"));
		}

		// Para no salir por wdg reset
		if ( (rcd_nbr % 64) == 0 ) {
			wdt_reset();
		}
	}

	FreeRTOS_write( &pdUART1, "\r\n\0", sizeof("\r\n\0"));
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_FF);

	// RESET
	u_reset();

}
//------------------------------------------------------------------------------------
uint8_t FF_errno( void )
{
	// Retorna el codigo de error de errno.
	return( FCB.ff_stat.errno);
}
//------------------------------------------------------------------------------------
static uint8_t pv_memChecksum( uint8_t *buff, uint8_t limit )
{
uint8_t checksum = 0;
uint8_t i;

	for( i=0; i<limit; i++)
		checksum += buff[i];

	checksum = ~checksum;
	return (checksum);
}
//----------------------------------------------------------------------------------
