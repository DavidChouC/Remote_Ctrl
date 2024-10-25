#ifndef __BSP_USART_H_
#define __BSP_USART_H_
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

int fputc(int ch, FILE *f);
extern uint8_t RxBuffer[1];

#endif
