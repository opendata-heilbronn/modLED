#pragma once

#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "globals.h"
#include "uart.h"
#include "main.h"
#include "display.h"

//these includes are correctly handled during platform io build
// they probably won't work with the stock makefile, changes are needed
#include "wizchip_conf.h"
#include "socket.h"
#include "DHCP/dhcp.h"

typedef struct {
    uint16_t opcode;
    uint8_t sequence;
    uint16_t universe;
    uint16_t dataLength;
    uint8_t* data;
} artnetPacket;


void initEthernet();
void initArtnet();
void loopArtnet();