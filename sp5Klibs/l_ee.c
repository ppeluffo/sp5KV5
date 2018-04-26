/*
 * ee_sp5K.c
 *
 *  Created on: 26/10/2015
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#include <l_ee.h>

//------------------------------------------------------------------------------------
uint8_t EE_read( uint16_t eeAddress, char *data, uint8_t length )
{
	// Lee en la EE, desde la posicion 'address', 'length' bytes
	// y los deja en el buffer apuntado por 'data'

	// No son funciones que se usen directamente por la aplicacion !!!
	// NO SE BLOQUEAN CON EL SEMAFORO. HAY QUE HACERLO EXPLICITAMENTE !!!
	//

uint8_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;

	// Lo primero es obtener el semaforo
	//FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_EE );
	// Luego indicamos el periferico i2c en el cual queremos leer

	val = EE_ADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val, DEBUG_EE);
	// Luego indicamos la direccion desde donde leer: largo ( 2 bytes )
	val = 2;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val, DEBUG_EE);
	// y direccion
	val = eeAddress;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_EE);

	// Por ultimo leemos.
	xBytes = length;
	xReturn = FreeRTOS_read(&pdI2C, data, xBytes);

#ifdef ee_DEBUG
//		snprintf_P( debug_printfBuff,CHAR128,PSTR("++DEBUG EE_READ: xBytes:0x%02x, xReturn:0x%02x\r\n\0"),xBytes,xReturn );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif

	// Y libero el semaforo.
	//FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_EE);

	return(xReturn);

}
//------------------------------------------------------------------------------------
uint8_t EE_write( uint16_t eeAddress, char *data, uint8_t length )
{
	// Escribe en la EE a partir de la posicion 'address', la cantidad
	// 'length' de bytes apuntados por 'data'
	// Puedo estar escribiendo un pageWrite entonces debo controlar no
	// salime de la pagina.
	//
	// No son funciones que se usen directamente por la aplicacion !!!
	// NO SE BLOQUEAN CON EL SEMAFORO. HAY QUE HACERLO EXPLICITAMENTE !!!
	//

uint8_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;
uint16_t n, pageBytesFree;

#ifdef ee_DEBUG
//		snprintf_P( debug_printfBuff,CHAR128,PSTR("++DEBUG EE_WRITE: eeAddress:0x%02x, length:0x%02x\r\n\0"),eeAddress,length );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
	// Lo primero es obtener el semaforo
	//FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_EE);
	// Luego indicamos el periferico i2c en el cual queremos leer

	val = EE_ADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val, DEBUG_EE);
	// Luego indicamos la direccion a partir de donde escribir: largo ( 2 bytes )
	val = 2;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val, DEBUG_EE);
	// y direccion
	val = eeAddress;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_EE);

	// Por ultimo escribimos xBytes.
	// Controlo no hacer page overflow
	xBytes = length;
//	n = length % EE_PAGESIZE;
//	pageBytesFree = (n+1)*EE_PAGESIZE - length;
//	if ( pageBytesFree < length ) {
//		xBytes = pageBytesFree;
//	}

	xReturn = FreeRTOS_write(&pdI2C, data, xBytes);

#ifdef ee_DEBUG
//		snprintf_P( debug_printfBuff,CHAR128,PSTR("++DEBUG EE_WRITE: eeAddress:0x%02x, xBytes:0x%02x, xReturn:0x%02x\r\n\0"),eeAddress,xBytes,xReturn );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif

	// Y libero el semaforo.
	//FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	return(xReturn);

}
//------------------------------------------------------------------------------------
bool EE_test_write(char *s0, char *s1)
{
uint8_t length = 0;
char *p;
bool retS = false;

	p = s1;
	while (*p != 0) {
		p++;
		length++;
	}
//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("S=[%s](%d)\r\n\0"),s1, length);
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_EE);

	if ( EE_write( (uint16_t)(atoi(s0)), s1, length ) == length ) {
		retS = true;
	}

	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_EE);
	return(retS);
}
//-----------------------------------------------------------------------------------
bool EE_test_read(char *s0, char *s1, char *s2)
{

bool retS = false;
uint8_t length = (uint8_t)(atoi(s2));

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_EE);


	if ( EE_read( (uint16_t)(atoi(s0)), s1, length ) == length ) {
		retS = true;
	}

	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_EE);
	return(retS);
}
//-----------------------------------------------------------------------------------
