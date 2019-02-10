#include "ethernet.h"

void spiStart() {
  __HAL_SPI_ENABLE(&ETH_SPI);
}

void spiStop() {
  // while(__HAL_SPI_GET_FLAG(&ETH_SPI, SPI_SR_BSY)); //maybe needed, dunno
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
  ipAssigned = true;
}

void cbkIPConflict() {
  printf("ERROR: IP conflict!\n");
}

//unused and untested for now
void doDHCP() {
  printf("Beginning DHCP...\n");
  uint8_t dhcp_buffer[1024];
  wiz_NetInfo netInfo = {
    .dhcp = NETINFO_DHCP
  };
  generateMAC(netInfo.mac);

  setSHAR(netInfo.mac);

  DHCP_init(SOCKET_DHCP, dhcp_buffer);

  reg_dhcp_cbfunc(cbkIPAssigned, cbkIPAssigned, cbkIPConflict);

  //should probably be called in main loop, but test for now if it even works at all
  uint32_t repeats = 10000;
  while(!ipAssigned && repeats > 0) {
    DHCP_run();
    repeats--;
  }

  if(!ipAssigned) {
    printf("ERROR: no IP was assigned in time \n");
    return;
  }

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

  wizchip_setnetinfo(&netInfo);
  
}

void initEthernet() {
  printf("Initializing Ethernet...");

  // Reset ethernet module. Does not improve communicatoin either...
//   HAL_GPIO_WritePin(ETH_RESET_PIN, 0);
//   HAL_Delay(1);
//   HAL_GPIO_WritePin(ETH_RESET_PIN, 1);
//   HAL_Delay(400);
  
  reg_wizchip_cs_cbfunc(&spiStart, &spiStop);
  reg_wizchip_spi_cbfunc(&spiReadByte, &spiWriteByte);
  reg_wizchip_spiburst_cbfunc(&spiRead, &spiWrite);

  uint8_t tx_size[8] = { 2, 2, 2, 2, 2, 2, 2, 2 }; // Device default memory setting
  uint8_t rx_size[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };

  //executes getMAC & the 3 IPs, afterwards writes 00 00 04 80 for reset
  // then setMAC & the 3 IPs, but I don't remember seeing that on the logicanalyzer, maybe because of the NULL pointers instead of 0?
  if(wizchip_init(tx_size, rx_size) < 0) { //initialize with default memory settings
    _Error_Handler(__FILE__, __LINE__);
  } 

  uint8_t mac[6];
  generateMAC(mac);
  setSHAR(mac);

  getSHAR(mac);

  printf(" done. MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  #ifdef STATIC_IP
    uint8_t ip[4] = { STATIC_IP };
    setSIPR(ip); //set client IP

    ip[3] = 1;
    setGAR(ip); //set gateway IP

    memset(ip, 0xFF, 3);
    ip[3] = 0;
    setSUBR(ip); //set subnet mask*/
  #else
    doDHCP();
  #endif
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