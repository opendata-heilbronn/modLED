#pragma once

#define DMA_TIMER   htim1
#define DMA_CHANNEL TIM_CHANNEL_1
#define PWM_TIMER   htim3
#define PWM_CHANNEL TIM_CHANNEL_1


#define PIN_OE  GPIOB, GPIO_PIN_4
#define PIN_LAT GPIOB, GPIO_PIN_5

#define PANEL_HEIGHT    8
#define PANEL_WIDTH     16
#define NUM_PIXELS      (PANEL_HEIGHT * PANEL_WIDTH)
#define PWM_RESOLUTION  6

#define DMA_BUF_LENGTH  ((NUM_PIXELS/2) * (1 << PWM_RESOLUTION))
uint8_t dmaBuf[DMA_BUF_LENGTH]; // buffer for the raw data to be shifted out per frame

// color mapping: [r1, r2, g, b]
uint32_t frameBuf[NUM_PIXELS];

#define RX_UART     huart1
#define RX_BAUD     500000
#define UART_BUFFER_LENGTH 64
#define UART_PROTOCOL_INIT 1
uint8_t uartBuffer[UART_BUFFER_LENGTH];
uint16_t uartRxCounter;

#define INITAL_BRIGHTNESS   255
#define GAMMA_STEPS         256
#define GAMMA_CORRECTION    2.2
uint8_t gammaTable[GAMMA_STEPS];
uint8_t globalBrightness;
uint8_t pwmStepIdx;