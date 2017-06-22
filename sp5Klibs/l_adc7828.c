/*
 * ads7828_sp5KFRTOS.c
 *
 *  Created on: 15/4/2015
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#include <l_adc7828.h>
#include <sp5KV5.h>

//-------------------------------------------------------------------------------------
bool ADC_read(uint8_t channel, uint16_t *value)
{

	// Lee un canal del conversor ADC (0..7)
	// No es el orden que representan en el datalogger !!!

uint8_t ads7828Channel;
uint8_t ads7828CmdByte;
uint8_t buffer[2];
uint16_t retValue;
size_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;
bool retS = false;

	if ( channel > 7) {
		goto quit;
	}
	// Convierto el canal 0-7 al C2/C1/C0 requerido por el conversor.
	ads7828Channel = (((channel>>1) | (channel&0x01)<<2)<<4) | ADS7828_CMD_SD;
	// do conversion
	// Armo el COMMAND BYTE
	ads7828CmdByte = ads7828Channel & 0xF0;	// SD=1 ( single end inputs )
	ads7828CmdByte |= ADS7828_CMD_PDMODE2;	// Internal reference ON, A/D converter ON

	// start conversion on requested channel
	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_ADC);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = ADS7828_ADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val, DEBUG_ADC);
	// Luego indicamos en que posicion del periferico queremos leer: largo
	val = 0;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val, DEBUG_ADC);
	// y direccion
	val = 0;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_ADC);

	// Escribo en el ADS
	xBytes = 1;
	xReturn = FreeRTOS_write(&pdI2C, &ads7828CmdByte, xBytes);
	if (xReturn != xBytes ) {
		goto quit;
	}

	// Espero el settle time
	vTaskDelay(1);

	// retrieve conversion result
	xBytes = 2;
	xReturn = FreeRTOS_read(&pdI2C, &buffer, xBytes);
	if (xReturn != xBytes ) {
		goto quit;
	}

	retValue = (buffer[0]<<8) | buffer[1];
	// pack bytes and return result
	*value = retValue;

	// Apago el conversor
//	ads7828CmdByte = ads7828Channel & 0xF0;
//	ads7828CmdByte |= ADS7828_CMD_PDMODE0;	// Internal reference OFF, A/D converter OFF
//	status = pvADS7828_write( &ads7828CmdByte);

	retS = true;
quit:
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_ADC);
	return (retS);

}
//------------------------------------------------------------------------------------
bool ADC_readDlgCh(uint8_t channel, uint16_t *adc_value)
{
	// Lee el ADC pero el canal pasado es el del datalogger, no el del conversor.

	switch(channel) {
	case 0:				// AIN0->ADC3;
		return ( ADC_read( 3, adc_value));
		break;
	case 1:				// AIN1->ADC5;
		return ( ADC_read( 5, adc_value));
		break;
	case 2:				// AIN2->ADC7;
		return ( ADC_read( 7, adc_value));
		break;
	case 3:				// BATT->ADC1;
		return ( ADC_read( 1, adc_value));
		break;
	default:
		return(false);
		break;
	}

}
//------------------------------------------------------------------------------------
bool ADC_test_read( char *s, uint16_t *adcRetValue )
{
	// read adc channel
	// channel: argv[2]

	*adcRetValue = 9999;
	return ( ADC_readDlgCh ( atoi(s), adcRetValue ) );

}
//------------------------------------------------------------------------------------

