/*
 * l_ina3221.c
 *
 *  Created on: 13 de oct. de 2017
 *      Author: pablo
 */

#include <l_ina3221.h>
#include "sp5KV5.h"

#ifdef SP5KV5_8CH

#define INA3221_VCC_SETTLE_TIME	500

static uint8_t pv_get_INA_address(uint8_t id);

//------------------------------------------------------------------------------------
bool INA3221_read( uint8_t devAddress, uint8_t regAddress, char *data, uint8_t length )
{
	// C/registro es de 2 bytes de informacion.

size_t xReturn = 0U;
uint8_t val = 0;
uint8_t xBytes = 0;

		// Lo primero es obtener el semaforo
		FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_INA3221);

		// Luego indicamos el periferico i2c en el cual queremos leer
		val = devAddress;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val, DEBUG_INA3221);
//		snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: devAddr=0x%02x\r\n\0"), val );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

		// Luego indicamos el registro desde donde leer: largo ( 1 bytes )
		val = 1;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val, DEBUG_INA3221);
//		snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: length=0x%02x\r\n\0"), val );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

		// y direccion
		val = regAddress;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_INA3221);
//		snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: regAddr=0x%02x\r\n\0"), val );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

		// Por ultimo leemos.( 2 bytes )
		xBytes = 2;
		xReturn = FreeRTOS_read(&pdI2C, data, xBytes);
//		snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: size=0x%02x\r\n\0"), xBytes );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

		// Y libero el semaforo.
		FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_INA3221);

		if (xReturn != xBytes ) {
			return ( false );
		}

		return(true);

}
//------------------------------------------------------------------------------------
bool INA3221_write( uint8_t devAddress, uint8_t regAddress, char *data, uint8_t length )
{
	// Escribe en el INA3221 en la posicion regAddress, la cantidad
	// 'length' de bytes apuntados por 'data'
	// En los INA3221 siempre se escriben solo 2 bytes de datos !!!
	//
size_t xReturn = 0U;
uint8_t val = 0;
uint8_t xBytes = 0;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_INA3221);

	// Luego indicamos el periferico i2c ( INA3221 ) en el cual queremos leer
	val = devAddress;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val, DEBUG_INA3221);

	// Luego indicamos la direccion ( el registro ) a partir de donde escribir: largo ( 1 bytes )
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val, DEBUG_INA3221);
	// y direccion ( registro )
	val = regAddress;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_INA3221);

	// Por ultimo escribimos xBytes.
	//xBytes = length;
	xBytes = 2;	// En los INA siempre son 2 bytes
	xReturn = FreeRTOS_write(&pdI2C, data, xBytes);

	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_INA3221);

	if (xReturn != xBytes ) {
		return ( false );
	}

	return(true);

}
//------------------------------------------------------------------------------------
bool INA3221_test_write(uint8_t id, char *s_regAddr, char *s_value )
{
	/* Se usa para testear desde el modo comando las funciones de escribir la ina.
	 * El unico registro que programamos es el de configuracion, 0.
	 */

uint8_t regAddress;
uint16_t devAddress;
uint16_t value;
char data[3];


	devAddress = pv_get_INA_address(id);
	regAddress = atoi(s_regAddr);
	value = atoi(s_value);
	memset(data,'\0', sizeof(data));
	data[0] = ( value & 0xFF00 ) >> 8;
	data[1] = ( value & 0x00FF );

	//snprintf_P( debug_printfBuff,128, PSTR("Raddr=%d, Val=0x%04x, d0=0x%02x,d1=0x%02x \r\n\0"),regAddress,value,data[0],data[1]);
	//FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

 	INA3221_write( devAddress, regAddress, data, 2 );

	return(true);
}
//-----------------------------------------------------------------------------------
uint16_t INA3221_test_read( uint8_t id, uint8_t regAddress )
{

	// Me devuelve el registro convertido a 16 bits.
	// A diferencia de INA3221_read que me devuelve los 2 bytes del registro, sin procesarlo.

uint8_t devAddress;
char res[3];
uint16_t val1;
//float mV;

//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("DEBUG3: id=%d, reg=%d\r\n"),id, regAddress );
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	devAddress = pv_get_INA_address(id);

//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("DEBUG4: dev=%d\r\n"),devAddress );
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	memset(res,'\0', sizeof(res));
	INA3221_read(devAddress, regAddress, res, 2 );

	val1 = 0;
	val1 = ( res[0]<< 8 ) + res[1];
	val1 = val1 >> 3;

	//mV = ( val1 / 1000.0 * 40 );

//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("DEBUG5: res0=%d, res1=%d, val1=%d\r\n"),res[0], res[1], val1 );
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	return(val1);
}
//-----------------------------------------------------------------------------------
static uint8_t pv_get_INA_address(uint8_t id)
{
	switch(id) {
	case 0:
		return(INA3221_ADDR_0);	// Canales 0,1,2
		break;
	case 1:
		return(INA3221_ADDR_1); // Canales 3,4,5
		break;
	case 2:
		return(INA3221_ADDR_2); // Canales 6,7,8
		break;
	}
	return(99);

}
//-----------------------------------------------------------------------------------
uint16_t pub_INA3221_read(char *s_inaId, char *s_inaReg)
{
	// Interface publica al driver INA3221 para leer un registro indicado por nombre.
	// read ina {0,1,2} {conf|chxshv|chxbusv|mfid|dieid}

uint8_t ina_id;

	ina_id = atoi(s_inaId);

//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("INA %d, REG=%s\r\n\0"),ina_id, strupr( s_inaReg ) );
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );


	if (!strcmp_P( strupr( s_inaReg ), PSTR("CONF\0"))) {
		return( INA3221_test_read( ina_id, INA3231_CONF_ADDR ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("CH1SHV\0"))) {
		return( INA3221_test_read( ina_id, INA3221_CH1_SHV ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("CH1BUSV\0"))) {
		return( INA3221_test_read( ina_id, INA3221_CH1_BUSV ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("CH2SHV\0"))) {
		return( INA3221_test_read( ina_id, INA3221_CH2_SHV ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("CH2BUSV\0"))) {
		return( INA3221_test_read( ina_id, INA3221_CH2_BUSV ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("CH3SHV\0"))) {
		return( INA3221_test_read( ina_id, INA3221_CH3_SHV ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("CH3BUSV\0"))) {
		return( INA3221_test_read( ina_id, INA3221_CH3_BUSV ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("MFID\0"))) {
		return( INA3221_test_read( ina_id, INA3221_MFID ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("DIEID\0"))) {
		return( INA3221_test_read( ina_id, INA3221_DIEID ) );
	}

	return(0);

}
//----------------------------------------------------------------------------------

#endif /* SP5KV5_8CH */
