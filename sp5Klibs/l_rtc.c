/*
 * sp5KFRTOS_rtc.c
 *
 *  Created on: 01/11/2013
 *      Author: root
 *
 * Funciones del RTC DS1340-33 modificadas para usarse con FRTOS.
 *
 *
 */
//------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#include "l_rtc.h"

static char pv_bcd2dec(char num);
static char pv_dec2bcd(char num);

//------------------------------------------------------------------------------------
// Funciones de uso general
//------------------------------------------------------------------------------------
bool RTC_read(RtcTimeType_t *rtc)
{
	// Retorna la hora formateada en la estructura RtcTimeType_t
	// No retornamos el valor de EOSC ni los bytes adicionales.

uint8_t oscStatus;
uint8_t data[8];
size_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_RTC);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = RTC_DEVADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val, DEBUG_RTC);
	// Luego indicamos en que posicion del periferico queremos leer: largo
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val, DEBUG_RTC);
	// y direccion
	val = 0;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_RTC);
	// Por ultimo leemos 8 bytes.
	xBytes = sizeof(data);
	xReturn = FreeRTOS_read(&pdI2C, &data, xBytes);
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_RTC);

	if (xReturn != xBytes ) {
		return ( false );
	}

	// Decodifico los resultados.
	oscStatus = 0;
	if ( (data[0] & 0x80) == 0x80 ) {		// EOSC
		oscStatus = 1;
	}
	rtc->sec = pv_bcd2dec(data[0] & 0x7F);
	rtc->min = pv_bcd2dec(data[1]);
	rtc->hour = pv_bcd2dec(data[2] & 0x3F);
	rtc->day = pv_bcd2dec(data[4] & 0x3F);
	rtc->month = pv_bcd2dec(data[5] & 0x1F);
	rtc->year = pv_bcd2dec(data[6]) + 2000;

	return(true);
}
//------------------------------------------------------------------------------------
bool RTC_write(RtcTimeType_t *rtc)
{
	// Setea el RTC con la hora pasada en la estructura RtcTimeType

uint8_t data[8];
size_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;

	data[0] = 0;	// EOSC = 0 ( rtc running)
	data[1] = pv_dec2bcd(rtc->min);
	data[2] = pv_dec2bcd(rtc->hour);
	data[3] = 0;
	data[4] = pv_dec2bcd(rtc->day);
	data[5] = pv_dec2bcd(rtc->month);
	data[6] = pv_dec2bcd(rtc->year);

	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_RTC);
	val = RTC_DEVADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val, DEBUG_RTC);
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH,&val, DEBUG_RTC);
	val = 0;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_RTC);

	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_RTC);

	if (xReturn != xBytes ) {
		return ( false );
	}

	return(true);
}
//------------------------------------------------------------------------------------
int8_t RTC_date_to_str(char *str, const uint8_t size)
{
	// Convierte los datos del RTC a un string con formato DD/MM/YYYY hh:mm:ss

RtcTimeType_t rtcDateTime;
int8_t pos = -1;

	if ( RTC_read(&rtcDateTime) ) {
		pos = snprintf( str, size ,"%02d/%02d/%04d %02d:%02d:%02d\r\n\0",rtcDateTime.day,rtcDateTime.month, rtcDateTime.year, rtcDateTime.hour,rtcDateTime.min, rtcDateTime.sec );
	}
	return(pos);

}
//------------------------------------------------------------------------------------
bool RTC_str_to_date(char *str)
{
char dateTimeStr[11];
char tmp[3];
bool retS;
RtcTimeType_t rtcDateTime;


	/* YYMMDDhhmm */
	if ( str == NULL )
		return(false);

		memcpy(dateTimeStr, str, 10);
		// year
		tmp[0] = dateTimeStr[0]; tmp[1] = dateTimeStr[1];	tmp[2] = '\0';
		rtcDateTime.year = atoi(tmp);
		// month
		tmp[0] = dateTimeStr[2]; tmp[1] = dateTimeStr[3];	tmp[2] = '\0';
		rtcDateTime.month = atoi(tmp);
		// day of month
		tmp[0] = dateTimeStr[4]; tmp[1] = dateTimeStr[5];	tmp[2] = '\0';
		rtcDateTime.day = atoi(tmp);
		// hour
		tmp[0] = dateTimeStr[6]; tmp[1] = dateTimeStr[7];	tmp[2] = '\0';
		rtcDateTime.hour = atoi(tmp);
		// minute
		tmp[0] = dateTimeStr[8]; tmp[1] = dateTimeStr[9];	tmp[2] = '\0';
		rtcDateTime.min = atoi(tmp);

		retS = RTC_write(&rtcDateTime);
		return(retS);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static char pv_dec2bcd(char num)
{
	// Convert Decimal to Binary Coded Decimal (BCD)
	return ((num/10 * 16) + (num % 10));
}
//------------------------------------------------------------------------------------
static char pv_bcd2dec(char num)
{
	// Convert Binary Coded Decimal (BCD) to Decimal
	return ((num/16 * 10) + (num % 16));
}
//------------------------------------------------------------------------------------
