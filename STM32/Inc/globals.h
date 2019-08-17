#pragma once
#include "stm32f1xx_hal.h"

#define DMA_TIMER       htim1
#define DMA_CHANNEL     TIM_CHANNEL_1 //PA08
#define PWM_TIMER       htim3
#define PWM_CHANNEL     TIM_CHANNEL_1 //PB04
#define LATCH_TIMER     htim2
#define LATCH_CHANNEL   TIM_CHANNEL_1 //PA15



#define PIN_OE  GPIOB, GPIO_PIN_4
#define PIN_LAT GPIOB, GPIO_PIN_5
#define PIN_LED GPIOC, GPIO_PIN_13

#define _WIZCHIP_           5500
#define ETH_SPI             hspi2
extern SPI_HandleTypeDef    hspi2;
#define SPI2_NSS            GPIOB, GPIO_PIN_12
#define SOCKET_DHCP         0
#define SOCKET_ARTNET       1
#define SOCKET_OPC          2
// #define STATIC_IP       192, 168, 178, 250 // comment out for DHCP
#define PORT_ARTNET         6454
#define MAX_DHCP_RETRIES    3

#define PANEL_PIXEL_NUM     128 // amount of pixels of single panel
#define PIXEL_HEIGHT        16  // number of rows across all panels
#define PIXEL_WIDTH         16
#define NUM_PIXELS          (PIXEL_HEIGHT * PIXEL_WIDTH)
#define PWM_RESOLUTION      7
#define MIN_PWM_PRESCALER   18  //TODO: attention, prescale will not fit into 16 bits, if PWM_RESOLUTION > 10
//minor glitching at PSC 9, further down glitching becomes worse, DMA is not fast enough to keep up (160ns minimum pulse time)

#define DMA_BUF_LENGTH  ((NUM_PIXELS/2) * PWM_RESOLUTION)
uint8_t dmaBuf[DMA_BUF_LENGTH]; // buffer for the raw data to be shifted out per frame

// color mapping: [r1, r2, g, b]
uint32_t frameBuf[NUM_PIXELS];

#define RX_UART             huart1
extern UART_HandleTypeDef   huart1;
#define RX_UART_DMA         hdma_usart1_rx
#define RX_BAUD             1250000
#define UART_BUFFER_LENGTH  16
#define UART_PROTOCOL_INIT  1
uint8_t uartBuffer[UART_BUFFER_LENGTH];
uint16_t uartRxCounter;
#define PRINTF_UART         RX_UART

#define INITAL_BRIGHTNESS   255
#define GAMMA_STEPS         256
#define GAMMA_CORRECTION    2.2
uint16_t gammaTable[GAMMA_STEPS];
uint8_t globalBrightness;
uint8_t pwmStepIdx;

extern void startDMA();