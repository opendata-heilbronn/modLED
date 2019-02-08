#include "uart.h"
#include <errno.h>
#include <sys/unistd.h>


// enable printf functionality on PRINTF_UART
int _write(int file, char *data, int len) {
  if((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
      errno = EBADF;
      return -1;
  }

  HAL_StatusTypeDef status = HAL_UART_Transmit(&PRINTF_UART, (uint8_t*)data, len, 1000);
  return (status == HAL_OK ? len : 0);
}

// legacy printf function
void UART_Printf(const char* fmt, ...) {
  char buf[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  HAL_UART_Transmit(&RX_UART, (uint8_t*)buf, strlen(buf), 1000);
  va_end(args);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if(huart == &RX_UART) {
    for(uint8_t i = 0; i < UART_BUFFER_LENGTH; i++) {
      if(uartRxCounter == 0) { // packet preamble
        if(uartBuffer[i] != UART_PROTOCOL_INIT) { //when received packet does not match preamble
          uartRxCounter--; // wait for next 
        }
      }
      else {
        uint16_t byteId = (uartRxCounter - 1);
        uint8_t val = uartBuffer[i];
        switch(byteId % 3) {
          case 0: frameBuf[byteId / 3] &= 0x0000FFFF; frameBuf[byteId / 3] |= (val << 24 | val << 16); break;
          case 1: frameBuf[byteId / 3] &= 0xFFFF00FF; frameBuf[byteId / 3] |= val << 8; break;
          case 2: frameBuf[byteId / 3] &= 0xFFFFFF00; frameBuf[byteId / 3] |= val; break;
        }
      }

      uartRxCounter++;
      if(uartRxCounter >= (NUM_PIXELS * 3) + 1)
        uartRxCounter = 0;
    }
  }
}