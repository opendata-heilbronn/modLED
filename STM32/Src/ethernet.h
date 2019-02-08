#pragma once

#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "globals.h"
#include "uart.h"
#include "main.h"

//these includes are correctly handled during platform io build
// they probably won't work with the stock makefile, changes are needed
#include "wizchip_conf.h"
#include "socket.h"
#include "DHCP/dhcp.h"


void initEthernet();