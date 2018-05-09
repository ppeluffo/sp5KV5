/*
 * l_drv8814.h
 *
 *  Created on: 21 de oct. de 2017
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_DRV8814_H_
#define SRC_SPX_LIBS_L_DRV8814_H_

#include <l_iopines_3CH.h>
#include "FRTOS-IO.h"
#include "stdio.h"

#include "sp5KV5_defs.h"

#ifdef SP5KV5_3CH
//------------------------------------------------------------------------------------

void DRV8814_pulse(char channel, char phase, uint16_t pulse_width);
bool DRV8814_test_pulse(char *s0, char *s1, char *s2);

#define DRV8814_open_valve_A() DRV8814_test_pulse("A", "+", "100")
#define DRV8814_close_valve_A() DRV8814_test_pulse("A", "-", "100")
#define DRV8814_open_valve_B() DRV8814_test_pulse("B", "+", "100")
#define DRV8814_close_valve_B() DRV8814_test_pulse("B", "-", "100")

#endif /* SP5KV5_3CH */

#endif /* SRC_SPX_LIBS_L_DRV8814_H_ */
