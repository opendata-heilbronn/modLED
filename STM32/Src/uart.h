#pragma once

#include "globals.h"
#include <stdarg.h>
#include <string.h>

void UART_Printf(const char* fmt, ...);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart);