/*
 *
 * git commit -a -m "beta 473293390 001"
 * git remote add REM_SP5KV4 https://github.com/ppeluffo/sp5KV4.git
 * git push -u REM_SP5KV4 master
 *
 * WATCHDOG:
 * Para hacer un mejor seguimiento de las fallas, agrego a c/estado un nro.
 * Por otro lado, el WDG lo manejo en modo interrupcion / reset de modo que ante
 * un problema, la interrupcion guarda el estado y luego se resetea.
 * Al arrancar, leo el estado y lo trasmito.
 *
 * Agrego en la funcion de espera de I2C un timeout de modo que salgo con FALSE y eso hace
 * que el resto de las funciones indique un error.
 * Esto en ppio. podria evitar un error de reset por wdg.
 *
 * !! Agregar el salir automaticamente luego de 30 mins del modo service.
 *
 * V5.0.8:
 * En tkAnalog incorpora una flag para saber si estoy en el primer frame.
 * Si es asi lo descarto ya que sino el caudal es 0 lo cual es incorrecto.
 *
 * V5.0.7:
 * Igual a la 5.0.6 pero al entrar en modo sleep bloquea la terminal con el LM365.
 *
 * V5.0.3:
 * 1- El timerDial debe funcionar en 32bits para permitir que disque 1 sola vez a dia.
 * 2- Elimino el CONTINUO/DISCRETO: Si timerDial < 600 se trabaja en modo continuo. Si es mayor
 * en modo discreto.
 * La terminal la trabajo siempre como en modo discreto.
 * 3- Elimino del tkAnalog el modo service/monitorframe/monitorSQE.
 * 4- En tkDigital elimino el control que solo sea en modo normal. Es siempre.
 *
 * V5.0.1:
 * Modificaciones mayores.
 * 1- Dejo de lado la implementacion de FSM ya que hace imposible de entender el programa.
 *    Programo de modo convencional
 *    Genero una capa de drivers y otra de librerias.
 *    La aplicacion accede al HW por medio de llamadas a las librerias y/o FRTOS-IO
 * 2- SALIDAS: no considero mas a consignas sino que uso el concepto de salidas, las cuales pueden
 *    operar en modo consigna o normales
 * 3- Reescribo toda la tkGPRS para hacelo mas modular. Es la unica que no tiene wdg.
 * 4- DIGITAL: considero que solo mido caudales con los pulsos y cambio la forma de calcular el caudal
 *    instantaneo para que sea mas exacto.
 *    El tema es que cuando hay muchos pulsos, el metodo de los pulsos es mejor. Cuando hay pocos, el
 *    metodo del tiempo es mejor.
 *    La solucion es usar ambos, cuando la cantidad de pulsos por intervalo es menor a 20 ( 1 pulso de error )
 *    es el 5%, uso tiempo, si es mayor uso pulsos.
 *
 * V4.1.6:
 * Cambio la velocidad del puerto serial de consola para que trabaje a 9600 y asi poder
 * usar el bootloader.
 *
 * V4.1.5:
 * - Al cambiar la consignar remotamente, no estabamos procesandolo.
 * - La consigna no aceptaba que la nocturna sea anterior a la diurna
 * - Incorporo un comando para poder resetearme.
 *
 * V4.1.4:
 * - Agrego un sensor de TILT conectado al PB0.
 * - Agrego una variable al systemVars que habilita o no el tilt ( a discar )
 *
 * V4.1.3:
 * - Agrego en el frame de datos un campo que indique la calidad del frame.
 *   Queda del tipo:
 *   CTL=106&ST=0&LINE=20140607,083358,pA>62.12,pB>62.12,pc>4.23,q0>103.28,v1>22.4,bt>11.29
 *
 * V4.1.2:
 * - Modifico la inicializacion de WDT siguiendo las recomendaciones de Atmel y verifico el
 *   funcionamiento de c/wdg.
 *   http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
 *   http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
 * - El WDT se prende por fuses grabandolos en 0xFF(low), 0xC9(high), 0xFD(extended)
 *   Esto me asegura que no se van a borrar por hw.
 * - Borro todo lo referente a DCD ya que no lo uso.
 * - Agrego a tkAnalog una funcion que chequea la consistencia de los timers en c/ciclo y si estan
 *   mal resetea al uC
 * - Idem en tkGprs.
 * - Tambien controlo no exceder 12hs sin discar.
 * - Elimino el control del pin de la terminal de tkDigital y lo dejo todo en tkControl.
 * - En tkControl agrego la rutina pv_modemControl para controlar que el modem no quede
 *   prendido mas de 10mins. Con esto si alguna rutina queda en loop, no apagaria el modem y
 *   podriamos agarrar el problema.
 *
 * V4.0.9:
 * El problema es que el ADC no se prende y la lectura de los canales es erronea.
 * La razon es que el MCP se resetea a default y por lo tanto no prende los sensores ni los 3.3V.
 * Esto es porque el pigtail del modem irradia energia e introduce un pulso que resetea al MCP.
 * Lo resolvemos en 3 flancos diferentes:
 * 1- En la rutina MCP_testAndSet ( con la que prendemos los 3.3V ), verificamos que el MCP este configurado
 * y sino lo reconfiguramos.
 * 2- Hay veces que esto no es suficiente ya que entre tr02 y tr06, si se resetea el MCP no nos damos
 * cuenta hasta que leemos el ADC.
 * Entonces en tr02 mandamos un mensaje a tkGPRS que no trasmita, y en tr06 que puede trasmitir.
 * Con esto controlo no irradiar energia que resetee al MCP mientras estoy poleando ( 15s ).
 * En tkGPRS lo chequeo antes de perdir una IP y antes de abrir un socket.
 * 3- En caso que nada funcione, en el modulo tkAnalog si una medida me da error ( lectura del ADC ), descarto
 * la medida y la sustituyo por el valor anterior.
 * Como la medida de los sensores se hace en 4 pasos, si el primero ( tr05) da error ( por estar apagado el ADC ),
 * puedo corregirlo y prenderlo.
 *
 * V4.1.0:
 * Modifico el manejo de consignas para incorporar una FSM
 * El mismo problema de desconfiguracion puede ocurrir al setear las consignas.
 * Para esto entonces fijo que para setear las consignas el modem deba estar apagado.
 * La tarea tkConsignas consulta el estado del modem con u_modemPwrStatus()
 *
 * ----------------------------------------------------------------------------------------------------------------
 */

#include <sp5KV5.h>

static void pv_initMPU(void);

//----------------------------------------------------------------------------------------
// http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
// http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
//
// Function Pototype
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

// Function Implementation
void wdt_init(void)
{
	// Leo el MCUSR para saber cual fue la causa del reset
	mcusr_mirror = MCUSR;
	// e inmediatamente lo pongo en 0 por si se resetea saber la causa
    MCUSR = 0;
    // y deshabilito el wdg para dejar que el micro arranque y no quede en un loop de resets
    //  wdt_disable();
    // Como los fusibles estan para que el WDG siempre este prendido, lo reconfiguro a 8s lo
    // antes posible
    wdt_enable(WDTO_8S);
    return;
}
//------------------------------------------------------------------------------------

int main(void)
{
unsigned int i,j;

	//----------------------------------------------------------------------------------------
	// Rutina NECESARIA para que al retornar de un reset por WDG no quede en infinitos resets.
	// Copiado de la hoja de datos.

	wdgStatus.resetCause = mcusr_mirror;
	// Genero un delay de 1s. para permitir que el micro se estabilize.
	// Lleva tiempo a los osciladores estabilizarse.
	for (i=0; i<1000; i++)
		for (j=0; j<1000; j++)
				;

	//----------------------------------------------------------------------------------------

	wdt_reset();
	IO_init_pines();
	pv_initMPU();
	FreeRTOS_open(pUART0, ( UART_RXFIFO + UART_TXQUEUE ));
	FreeRTOS_open(pUART1, ( UART_RXFIFO + UART_TXQUEUE ));
	FreeRTOS_open(pI2C, 0);

	/* Arranco el RTOS. */
	startTask = false;

	// Creo los semaforos
	sem_SYSVars = xSemaphoreCreateMutex();

	// Inicializacion de modulos de las tareas que deben hacerce antes
	// de arrancar el FRTOS
	tkAnalogInit();

	// Creo las tasks
	xTaskCreate(tkCmd, "CMD", tkCmd_STACK_SIZE, NULL, tkCmd_TASK_PRIORITY,  &xHandle_tkCmd);
	xTaskCreate(tkDigitalIn, "DIN", tkDigitalIn_STACK_SIZE, NULL, tkDigitalIn_TASK_PRIORITY,  &xHandle_tkDigitalIn);
	xTaskCreate(tkControl, "CTL", tkControl_STACK_SIZE, NULL, tkControl_TASK_PRIORITY,  &xHandle_tkControl);
	xTaskCreate(tkAnalogIn, "AIN", tkAIn_STACK_SIZE, NULL, tkAIn_TASK_PRIORITY,  &xHandle_tkAIn);
	xTaskCreate(tkGprsTx, "GPRS", tkGprs_STACK_SIZE, NULL, tkGprs_TASK_PRIORITY,  &xHandle_tkGprs);
	xTaskCreate(tkGprsRx, "GPRX", tkGprsRx_STACK_SIZE, NULL, tkGprsRx_TASK_PRIORITY,  &xHandle_tkGprsRx);
	xTaskCreate(tkOutputs, "OUTS", tkOutputs_STACK_SIZE, NULL, tkOutputs_TASK_PRIORITY,  &xHandle_tkOutputs);

	systemWdg = WDG_CTL + WDG_CMD + WDG_DIN + WDG_AIN + WDG_OUT;

	/* Arranco el RTOS. */
	vTaskStartScheduler();

	// En caso de panico, aqui terminamos.
	exit (1);
}
/*------------------------------------------------------------------------------------*/
static void pv_initMPU(void)
{
	// Son acciones que se hacen antes de arrancar el RTOS

	// Configuro el modo de Sleep.
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
}
/*------------------------------------------------------------------------------------*/
void vApplicationIdleHook( void )
{

	for(;;) {

		if ( ( u_modem_prendido() == false ) && ( u_terminal_is_on() == false) && ( MODO_DISCRETO )) {
			sleep_mode();
		}
	}
}
/*------------------------------------------------------------------------------------*/
void vApplicationTickHook( void )
{
	// Para contar tiempo en forma exacta, llevo este contador de ticks.
	ticks++;
}
/*------------------------------------------------------------------------------------*/
