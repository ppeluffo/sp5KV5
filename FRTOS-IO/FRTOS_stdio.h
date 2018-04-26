/*
 * FRTOS_stdio.h
 *
 *  Created on: 23 de abr. de 2018
 *      Author: pablo
 */

#ifndef FRTOS_IO_FRTOS_STDIO_H_
#define FRTOS_IO_FRTOS_STDIO_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
//#include "printf.h"
//----------------------------------------------------------------------------
// buffer size used omly for printf (created on stack)
//#define PRINTF_BUFFER_SIZE        128U

// ntoa conversion buffer size, this must be big enough to hold
// one converted numeric number including padded zeros (created on stack)
#define PRINTF_NTOA_BUFFER_SIZE    32U

// ftoa conversion buffer size, this must be big enough to hold
// one converted float number including padded zeros (created on stack)
#define PRINTF_FTOA_BUFFER_SIZE    32U

// define this to support floating point (%f)
#define PRINTF_FLOAT_SUPPORT

// define this to support long long types (%llu or %p)
//#define PRINTF_LONG_LONG_SUPPORT

int FRTOS_sprintf(char* buffer, const char* format, ...);
int FRTOS_snprintf(char* buffer, size_t count, const char* format, ...);
int FRTOS_snprintf_P(char* buffer, size_t count, const char* format, ...);

#endif /* FRTOS_IO_FRTOS_STDIO_H_ */
