/*------------------------------------------------------------------------------------
 * rtc_sp5KFRTOS.h
 * Autor: Pablo Peluffo @ 2015
 * Basado en Proycon AVRLIB de Pascal Stang.
 *
 * Son funciones que impelementan la API de acceso al RTC del sistema SP5K con FRTOS.
 * Para su uso debe estar inicializado el semaforo del bus I2C, que se hace llamando a i2cInit().
 *
 *
*/

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#ifndef AVRLIBFRTOS_RTC_SP5KFRTOS_H_
#define AVRLIBFRTOS_RTC_SP5KFRTOS_H_

#include <avr/pgmspace.h>

#include "FRTOS-IO.h"
#include "sp5K_i2c.h"

//#define rtc_DEBUG

#ifdef rtc_DEBUG
	#define DEBUG_RTC 6
#else
	#define DEBUG_RTC 0
#endif

// Direccion del bus I2C donde esta el DS1340
#define RTC_DEVADDR		   	0xD0

typedef struct
{
	// Tamanio: 7 byte.
	// time of day
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	// date
	uint8_t day;
	uint8_t month;
	uint16_t year;

} RtcTimeType_t;


bool RTC_read(RtcTimeType_t *rtc);
bool RTC_write(RtcTimeType_t *rtc);

int8_t RTC_date_to_str(char *str, const uint8_t size);
bool RTC_str_to_date(char *str);


#endif /* AVRLIBFRTOS_RTC_SP5KFRTOS_H_ */
