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

enum ethernetStates {
    ETH_DHCP_FAILED = -2,
    ETH_INIT_FAILED = -1,
    ETH_UNINITIALIZED = 0,
    ETH_INIT_DONE,
    ETH_DHCP_STARTED,
    ETH_IP_ASSIGNED,
} volatile ethState, prevEthState;

void initEthernet();
void loopEthernet();
void initArtnet();
void loopArtnet();

uint8_t dhcpRetries;
uint32_t lastDhcpTick, dhcpRuns, dhcpStarted;