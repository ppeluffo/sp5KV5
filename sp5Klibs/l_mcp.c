/*
 *  sp5KFRTOS_mcp.c
 *
 *  Created on: 01/11/2013
 *      Author: root
 *
 *  Funciones del MCP23008 modificadas para usarse con FRTOS.
 */
//------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#include <l_mcp.h>
#include <sp5KV5.h>
#include "FRTOS-IO.h"

// Funciones privadas del modulo MCP
static void pvMCP_checkConfiguration(void);
static void pvMCP_init_MCP0(void);
static void pvMCP_init_MCP1(void);
//
//------------------------------------------------------------------------------------
// Funciones de uso general
//------------------------------------------------------------------------------------
void MCP_init(uint8_t device_id )
{
	// inicializo los MCP para la configuracion de pines del HW sp5K.
	// Como accedo al bus I2C, debo hacerlo una vez que arranco el RTOS.

	// Inicializo los pines del micro como entradas para las interrupciones del MCP.
	cbi(MCP0_DDR, MCP0_BIT);
	cbi(MCP1_DDR, MCP1_BIT);

	switch(device_id) {
	case 0:
		pvMCP_init_MCP0();
		break;
	case 1:
		pvMCP_init_MCP1();
		break;
	}

}
//------------------------------------------------------------------------------------
bool MCP_write( uint8_t deviceId, uint8_t byteAddr, uint8_t value )
{
uint8_t regValue;
size_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_MCP);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = deviceId;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val, DEBUG_MCP);
	// Luego indicamos la direccion del dispositivo: largo
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val, DEBUG_MCP);
	// y direccion
	val = byteAddr;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	// Por ultimo indico la direccion interna a leer
	// Por ultimo leemos 1 byte.
	xBytes = 1;
	regValue = value;
	xReturn = FreeRTOS_write(&pdI2C, &regValue, xBytes);
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_MCP);

	if (xReturn != xBytes ) {
		return ( false );
	}

	return(true);

}
//------------------------------------------------------------------------------------
bool MCP_read( uint8_t deviceId, uint8_t byteAddr, uint8_t *retValue )
{
size_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_MCP);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = deviceId;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val, DEBUG_MCP);
	// Luego indicamos en que posicion del periferico queremos leer: largo
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val, DEBUG_MCP);
	// y direccion
	val = byteAddr;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	// Por ultimo leemos 1 byte.
	xBytes = 1;
	xReturn = FreeRTOS_read(&pdI2C, retValue, xBytes);
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_MCP);

	if (xReturn != xBytes ) {
		return ( false );
	}

	return(true);

}
//-----------------------------------------------------------------------------------
bool MCP_modify( uint8_t deviceId, uint8_t byteAddr, uint8_t value, uint8_t bitMask )
{
size_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;
uint8_t regValue;
bool retS = false;

//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("MCPT&S [dev=%d][addr=%d][val=%d][msk=%d].\r\n\0"),deviceId, byteAddr, value,bitMask );
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	// CONTROLO QUE SI EL MCP ES 23018 ESTE BIEN CONFIGURADO ANTES
	if ( deviceId == MCP1_ADDR ) {
		pvMCP_checkConfiguration();
	}

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_MCP);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = deviceId;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val, DEBUG_MCP);
	// Luego indicamos cuantos bytes queremos leer del dispositivo: largo.
	// En los MCP se lee y escribe de a 1 registro.
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val, DEBUG_MCP);
	// Ahora indicamos desde que posicion queremos leer: direccion
	val = byteAddr;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	// Por ultimo leemos.
	xBytes = 1;
	xReturn = FreeRTOS_read(&pdI2C, &regValue, xBytes);

	if (xReturn != xBytes ) {
		goto quit;
	}

//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("MCPT&S [rdVal=%d]\r\n\0"),regValue );
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	// Modifico el registro
	if  (value == 0) {
		regValue &= ~BV(bitMask);
	} else {
		regValue |= BV(bitMask);
	}

	// Escribo en el MCP
	xBytes = 1;
	xReturn = FreeRTOS_write(&pdI2C, &regValue, xBytes);
	if (xReturn != xBytes ) {
		goto quit;
	}

//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("MCPT&S [wrVal=%d]\r\n\0"),regValue );
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	retS = true;

quit:
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_MCP);
	return(retS);
}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static void pvMCP_init_MCP0(void)
{
	// inicializa el MCP23008 de la placa de logica
	// NO CONTROLO ERRORES.

uint8_t data;
size_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_MCP);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = MCP0_ADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val, DEBUG_MCP);
	// Luego indicamos en que posicion del periferico queremos leer: largo
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val, DEBUG_MCP);
	// y direccion

	// MCP0_IODIR: inputs(1)/outputs(0)
	val = MCP0_IODIR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	data = 0;
	data |= ( BV(MCP0_GPIO_IGPRSDCD) | BV(MCP0_GPIO_IGPRSRI) );
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// MCP0_IPOL: polaridad normal
	val = MCP0_IPOL;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	data = 0;
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// MCP0_GPINTEN: inputs interrupt on change.
	val = MCP0_GPINTEN;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	data = 0;
	//data |= ( BV(MCP_GPIO_IGPRSDCD) | BV(MCP_GPIO_IGPRSRI) | BV(MCP_GPIO_ITERMPWRSW) );
	data |=  BV(MCP0_GPIO_IGPRSDCD);
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// MCP0_INTCON: Compara contra su valor anterior
	val = MCP0_INTCON;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	data = 0;
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// MCP0_IOCON: INT active H
	val = MCP0_IOCON;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	data = 2;
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// MCP0_GPPU: pull-ups
	// Habilito los pull-ups en DCD
	val = MCP0_GPPU;
//	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	data = 0;
	xBytes = sizeof(data);
//	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// TERMPWR ON
	// Al arrancar prendo la terminal para los logs iniciales.
	val = MCP0_OLAT;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	data = 0;
	data |= BV(MCP0_GPIO_OTERMPWR);	// TERMPWR = 1
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_MCP);

	FreeRTOS_write( &pdUART1, "MCP0 init OK\r\n\0", sizeof("MCP0 init OK\r\n\0") );
}
//------------------------------------------------------------------------------------
static void pvMCP_init_MCP1(void)
{
	// Inicializo el MCP23018 de la placa analogica
	// El modo 0 indica uso normal, el modo 1 uso por reconfiguracion por lo que el log es por D_DEBUG

uint8_t data;
size_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;
static uint8_t modo = 0;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL, DEBUG_MCP);
	val = MCP1_ADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val, DEBUG_MCP);

	// IOCON
	val = MCP1_IOCON;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	data = 0x63; // 0110 0011
	//                      1->INTCC:Read INTCAP clear interrupt
	//                     1-->INTPOL: INT out pin active high
	//                    0--->ORDR: Active driver output. INTPOL set the polarity
	//                   0---->X
	//                 0----->X
	//                1------>SEQOP: sequential disabled. Address ptr does not increment
	//               1------->MIRROR: INT pins are ored
	//              0-------->BANK: registers are in the same bank, address sequential
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	//
	// DIRECCION
	// 0->output
	// 1->input
	val = MCP1_IODIRA;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	data = 0x80; // 1000 0000 ( GPA0..GPA6: outputs, GPA7 input )
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	val = MCP1_IODIRB;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	data = 0x64; // 0110 0100
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	//
	// PULL-UPS
	// 0->disabled
	// 1->enabled
	// Los dejo en 0 para ahorrar ma.
	val = MCP1_GPPUA;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	data = 0x00; // 1111 1111
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	val = MCP1_GPPUB;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	data = 0xF3; // 1111 0000
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	//
	// Valores iniciales de las salidas en 0
	val = MCP1_OLATA;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	data = 0x00;
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	val = MCP1_OLATB;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	data = 0x00;
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	//
	// GPINTEN: inputs interrupt on change.
	// Habilito que DIN0/1 generen una interrupcion on-change.
	// El portA no genera interrupciones
//	val = MCP1_GPINTENA;
//	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
//	data = 0;
//	xBytes = sizeof(data);
//	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	//data = 0x60; // 0110 0000
	//data |= ( BV(MCP1_GPIO_DIN0) | BV(MCP1_GPIO_DIN1) );
//	val = MCP1_GPINTENB;
//	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
//	data = 0;
//	xBytes = sizeof(data);
//	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	//
	// DEFVALB: valores por defecto para comparar e interrumpir
	//data = 0;
	//status = pvMCP_write( MCP1_DEFVALB, MCP_ADDR2, 1, &data);

	// INTCON: controlo como comparo para generar la interrupcion.
	// Con 1, comparo contra el valor fijado en DEFVAL
	// Con 0 vs. su valor anterior.
//	val = MCP1_INTCONB;
//	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
	//data |= ( BV(MCP1_GPIO_DIN0) | BV(MCP1_GPIO_DIN1) );
//	data = 0;
//	xBytes = sizeof(data);
//	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	// Borro interrupciones pendientes
//	val = MCP1_INTCAPB;
//	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val, DEBUG_MCP);
//	data = 0;
//	xBytes = sizeof(data);
//	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL, DEBUG_MCP);

	if ( modo == 0 ) {
		FreeRTOS_write( &pdUART1, "MCP1 init OK\r\n\0", sizeof("MCP1 init OK\r\n\0") );
		modo = 1;
	} else {
		snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("**DEBUG:: MCP1 init !!\r\n\0"));
		u_debugPrint(D_DEBUG, debug_printfBuff, sizeof(debug_printfBuff) );
	}
}
//------------------------------------------------------------------------------------
static void pvMCP_checkConfiguration(void)
{
bool retS = false;
uint8_t regValue;

	// Si estoy aqui es porque algo fallo. Verifico la configuracion del MCP
	retS = MCP_read( MCP1_ADDR, 0x01, &regValue );
	if ( !retS ) {
		// No pude leer el MCP. Asumo un problema en el bus.
		// Deshabilito la interfase TWI. La proxima lectura la voy a habilitar y veremos
		// si esto soluciona el problema.
		TWCR &= ~(1 << TWEN);
		snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("**DEBUG::pvMCP I2C BUS ERROR !! Disable interface...\r\n\0"));
		u_debugPrint(D_DEBUG, debug_printfBuff, sizeof(debug_printfBuff) );
		return;
	}

	// El bus I2C anda bien y el problema esta en el ADC o en el MCP
	if ( regValue != 0x64) {
		// El MCP esta desconfigurado: Lo reprogramo
		snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("**DEBUG::pvMCP MCP ERROR !! Reconfigure...\r\n\0"));
		u_debugPrint(D_DEBUG, debug_printfBuff, sizeof(debug_printfBuff) );
		pvMCP_init_MCP1();
		taskYIELD();
	}
}
//------------------------------------------------------------------------------------


