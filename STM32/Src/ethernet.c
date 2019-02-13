#include "ethernet.h"

void spiStart() {
  __HAL_SPI_ENABLE(&ETH_SPI);
}

void spiStop() {
  __HAL_SPI_DISABLE(&ETH_SPI);
}

void spiWrite(uint8_t* buf, uint16_t len) {
  HAL_SPI_Transmit(&ETH_SPI, buf, len, HAL_MAX_DELAY);
}

void spiRead(uint8_t* buf, uint16_t len) {
  HAL_SPI_Receive(&ETH_SPI, buf, len, HAL_MAX_DELAY);
}

void spiWriteByte(uint8_t byte) {
  spiWrite(&byte, 1);
}

uint8_t spiReadByte() {
  uint8_t byte;
  spiRead(&byte, 1);
  return byte;
}


void generateMAC(uint8_t* macArray) {
  uint32_t uid[3];
  HAL_GetUID(uid);
  macArray[0] = 0x42;
  macArray[1] = (uid[0] >> 0) & 0xFF;
  macArray[2] = (uid[0] >> 8) & 0xFF;
  macArray[3] = (uid[0] >> 16) & 0xFF;
  macArray[4] = (uid[0] >> 24) & 0xFF;
  macArray[5] = (uid[1] >> 0) & 0xFF;
}

volatile bool ipAssigned = false;

void cbkIPAssigned() {
  printf("IP assigned!\n");

  wiz_NetInfo netInfo = {
    .dhcp = NETINFO_DHCP
  };

  getSHAR(netInfo.mac);
  getIPfromDHCP(netInfo.ip);
  getGWfromDHCP(netInfo.gw);
  getSNfromDHCP(netInfo.sn);
  getDNSfromDHCP(netInfo.dns);

  printf("IP:  %d.%d.%d.%d\nGW:  %d.%d.%d.%d\nNet: %d.%d.%d.%d\nDNS: %d.%d.%d.%d\n",
    netInfo.ip[0], netInfo.ip[1], netInfo.ip[2], netInfo.ip[3],
    netInfo.gw[0], netInfo.gw[1], netInfo.gw[2], netInfo.gw[3],
    netInfo.sn[0], netInfo.sn[1], netInfo.sn[2], netInfo.sn[3],
    netInfo.dns[0], netInfo.dns[1], netInfo.dns[2], netInfo.dns[3]
  );

  clearDisplay();
  dispPrintf(0, 4, RED, "%.3d.", netInfo.ip[1]);
  dispPrintf(0, 9, GREEN, "%.3d.", netInfo.ip[2]);
  dispPrintf(0, 14, BLUE, "%.3d", netInfo.ip[3]);
  updateDisplay();

  wizchip_setnetinfo(&netInfo);

  ipAssigned = true;
  prevEthState = ethState;
  ethState = ETH_IP_ASSIGNED;
}

void cbkIPConflict() {
  printf("ERROR: IP conflict!\n");
}


void doDHCP() {
  prevEthState = ethState;
  ethState = ETH_DHCP_STARTED;
  dhcpStarted = HAL_GetTick();

  printf("Beginning DHCP...\n");
  clearDisplay();
  dispPrintf(0, 5, WHITE, "DHCP");
  updateDisplay();

  uint8_t dhcp_buffer[1024];
  DHCP_init(SOCKET_DHCP, dhcp_buffer);

  reg_dhcp_cbfunc(cbkIPAssigned, cbkIPAssigned, cbkIPConflict);
}

void initEthernet() {
  lastDhcpTick = 0;
  dhcpRuns = 0;
  
  ethState = ETH_UNINITIALIZED;
  prevEthState = ethState;
  printf("Initializing Ethernet...");
  
  reg_wizchip_cs_cbfunc(&spiStart, &spiStop);
  reg_wizchip_spi_cbfunc(&spiReadByte, &spiWriteByte);
  reg_wizchip_spiburst_cbfunc(&spiRead, &spiWrite);

  uint8_t tx_size[8] = { 2, 2, 2, 2, 2, 2, 2, 2 }; // Device default memory setting
  uint8_t rx_size[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };

  if(wizchip_init(tx_size, rx_size) < 0) { //initialize with default memory settings
    _Error_Handler(__FILE__, __LINE__);
  } 

  uint8_t mac[6];
  generateMAC(mac);
  setSHAR(mac);

  uint8_t savedMac[6];
  getSHAR(savedMac);

  if(memcmp(mac, savedMac, sizeof(mac)) != 0) {
    printf(" FAILED. Wrong MAC received back. Check SPI connections.\n");
    drawRect(0, 7, 16, 5, BLACK);
    dispPrintf(0, 7+4, BLUE, "E");
    dispPrintf(4, 7+4, RED, "ERR");
    updateDisplay();
    return;
  }

  printf(" done. MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  prevEthState = ethState;
  ethState = ETH_INIT_DONE;

  #ifdef STATIC_IP
    uint8_t ip[4] = { STATIC_IP };
    setSIPR(ip); //set client IP

    ip[3] = 1;
    setGAR(ip); //set gateway IP

    memset(ip, 0xFF, 3);
    ip[3] = 0;
    setSUBR(ip); //set subnet mask*/

    prevEthState = ethState;
    ethState = ETH_IP_ASSIGNED;
  #endif
}

void loopEthernet() {

  switch(ethState) {
    case ETH_INIT_DONE:
      doDHCP();
      break;

    case ETH_DHCP_STARTED:
      // if successful, sets ethState in DHCP callback
      if (DHCP_run() == DHCP_FAILED) {
        prevEthState = ethState;
        ethState = ETH_DHCP_FAILED;
      }
      break;

    case ETH_DHCP_FAILED:
      printf("ERROR: no IP was assigned in time \n");
      drawRect(0, 7, 16, 5, BLACK);
      dispPrintf(3, 7+4, RED, "ERR");
      updateDisplay();
      doDHCP(); // restart DHCP loop
      break;

    case ETH_IP_ASSIGNED:
      if(prevEthState != ETH_IP_ASSIGNED) { // if freshly initialized
        #ifdef SOCKET_ARTNET
          initArtnet();
        #endif
      }

      loopArtnet();
      prevEthState = ethState;
      break;

    case ETH_UNINITIALIZED:
    case ETH_INIT_FAILED:
      break;
  }


  if(HAL_GetTick() > lastDhcpTick + 1000) {
    lastDhcpTick = HAL_GetTick();
    DHCP_time_handler();

    if(ethState == ETH_DHCP_STARTED) {
      uint8_t dot = (HAL_GetTick() - dhcpStarted - 500) / 1000;
      drawPixel(dot % 10, 8 + dot / 10, GREEN);
    }
  }
}






#ifdef SOCKET_ARTNET
#define ARTNET_HEADER_LENGTH 18
#define PIXEL_DATA_LENGTH (PANEL_PIXEL_NUM * 3)
#define ARTNET_PACKAGE_LENGTH (ARTNET_HEADER_LENGTH + PIXEL_DATA_LENGTH)
uint8_t artnetBuf[ARTNET_PACKAGE_LENGTH];

void initArtnet() {
    printf("Creating ArtNet listener...");
    int8_t status = socket(SOCKET_ARTNET, Sn_MR_UDP, PORT_ARTNET, 0);
    if(status < 0) {
        printf("\nERROR creating ArtNet socket. Error code: %d\n", status);
        return;
    }
    printf(" done.\n");
}

void artnetCallback(artnetPacket packet) {
    uint8_t offset = packet.universe == 1 ? 0 : PANEL_PIXEL_NUM;
    uint8_t* d = packet.data;
    for(uint8_t i = 0; i < PANEL_PIXEL_NUM; i++) {
        uint16_t c = i * 3;
        frameBuf[i + offset] = d[c] << 24 | d[c] << 16 | d[c+1] << 8 | d[c+2];
    }
}

void loopArtnet() {
    if(getSn_RX_RSR(SOCKET_ARTNET) > 0) {
        uint8_t dstIp[4];
        uint16_t dstPort;
        int32_t rStatus = recvfrom(SOCKET_ARTNET, artnetBuf, ARTNET_PACKAGE_LENGTH, dstIp, &dstPort);
        if(rStatus < 0) {
            printf("ERROR receiving UDP package. Error code: %ld\n", rStatus);
        }

        if(dstPort == PORT_ARTNET) {
            artnetPacket packet;
            //check if ArtNrt header present
            if(strncmp((const char*)artnetBuf, "Art-Net\0", 8) != 0) {
                return;
            }
            packet.opcode = artnetBuf[8] | artnetBuf[9] << 8;
            if(packet.opcode == 0x5000) { // check if opcode equals Art-Net opcode
                packet.sequence = artnetBuf[12];
                packet.universe = artnetBuf[14] | artnetBuf[15] << 8;
                packet.dataLength = artnetBuf[17] | artnetBuf[16] << 8;
                packet.data = &artnetBuf[ARTNET_HEADER_LENGTH];

                artnetCallback(packet);
            }
        }
    }
}

#endif