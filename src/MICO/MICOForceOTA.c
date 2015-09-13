#include "mico.h"
#include "MICONotificationCenter.h"
#include "tftp/tftp.h"

#define DEFAULT_OTA_AP "MICO_OTA_AP"
#define DEFAULT_OTA_NETMASK "255.0.0.0"
#define DEFAULT_OTA_SERVER "10.0.0.2"
#define UDP_PORT 20000
#define TCP_PORT 30000

#define fota_log(M, ...) custom_log("Force OTA", M, ##__VA_ARGS__)
#define fota_log_trace() custom_log_trace("Force OTA")

extern void wlan_get_mac_address(char *mac);
extern void mico_write_ota_tbl(int len);

static int wifi_up = 0;

enum {
    OTA_SUCCESS = 0,
    OTA_NO_AP = -1,
    OTA_NO_FILE = -2,
    OTA_MD5_FAIL = -3,
    OTA_NO_MEM = -4,
};
/* Call back for OTA finished */
__weak void mico_ota_finished(int result, uint8_t *reserved)
{
    switch(result) {
    case OTA_SUCCESS:
        printf("OTA SUCCESS. Rebooting...\r\n");
        MicoSystemReboot();
        break;
    case OTA_NO_AP:
        printf("OTA FAIL. Can't find the OTA AP\r\n");
        break;
    case OTA_NO_FILE:
        printf("OTA FAIL. Can't find the OTA image\r\n");
        break;
    case OTA_MD5_FAIL:
        printf("OTA FAIL. MD5 check failed\r\n");
        break;
    case OTA_NO_MEM:
        printf("OTA FAIL. Don't have enough memory\r\n");
    default:
        break;
    }
}

static void FOTA_WifiStatusHandler(WiFiEvent event, void * const inContext)
{
  switch (event) {
  case NOTIFY_STATION_UP:
    wifi_up = 1;
    break;
  case NOTIFY_STATION_DOWN:
    wifi_up = 0;
    break;
 
  default:
    break;
  }

  return;
}

/* connect to AP: ssid="mico_ota_ap", security=OPEN.
  * Broadcast to find OTA server
  * Connect to OTA server, request to OTA.
  */
void mico_force_ota(void)
{
    network_InitTypeDef_st conf;
    tftp_file_info_t fileinfo;
    uint32_t ipaddr = inet_addr(DEFAULT_OTA_SERVER), flashaddr;
    int filelen, maxretry = 5, len, left, i = 0;
    uint8_t md5_recv[16];
    uint8_t md5_calc[16];
    uint8_t *tmpbuf;
    md5_context ctx;
    uint8_t mac[6], sta_ip_addr[16];
    
#define TMP_BUF_LEN 1024

    fota_log("Start OTA");
    MICORemoveAllNotification(mico_notify_WIFI_STATUS_CHANGED);
    MICORemoveAllNotification(mico_notify_WiFI_PARA_CHANGED);
    MICORemoveAllNotification(mico_notify_DHCP_COMPLETED);
    MICORemoveAllNotification(mico_notify_WIFI_CONNECT_FAILED);
	  MICORemoveAllNotification(mico_notify_EASYLINK_WPS_COMPLETED);
    MICOAddNotification( mico_notify_WIFI_STATUS_CHANGED, (void *)FOTA_WifiStatusHandler );
    micoWlanStopEasyLink();
	  micoWlanStopEasyLinkPlus();
		micoWlanStopAirkiss();
	  msleep(10);
		
    tmpbuf = (uint8_t*)malloc(TMP_BUF_LEN);
    if (tmpbuf == NULL) {
        fota_log("ERROR!! Can't get enough memory");
        mico_ota_finished(OTA_NO_MEM, NULL);
        return;
    }
    
    wlan_get_mac_address((char*)mac);
    
    sprintf((char*)sta_ip_addr, "10.%d.%d.%d", 
        mac[3], mac[4], mac[5]);
        
    fota_log("Staic IP = %s", sta_ip_addr);  
    
    memset(&conf, 0, sizeof(network_InitTypeDef_st));
    
    conf.wifi_mode = Station;
    strcpy(conf.wifi_ssid, DEFAULT_OTA_AP);
    
    conf.dhcpMode = DHCP_Disable;
    strcpy(conf.net_mask, DEFAULT_OTA_NETMASK);
    strcpy(conf.local_ip_addr, (const char*)sta_ip_addr);
    
    wifi_up = 0;
    fota_log("Connect to AP %s...", DEFAULT_OTA_AP);
    micoWlanStart(&conf);

    while(wifi_up == 0) {
        msleep(100);
        i++;
        if (i > 100) {
            fota_log("ERROR!! Can't find the OTA AP");
            mico_ota_finished(OTA_NO_AP, NULL);
            return;
        }
    }
    fota_log("AP connected, tftp download image...");

    fileinfo.filelen = UPDATE_FLASH_SIZE-1;
    fileinfo.flashaddr = UPDATE_START_ADDRESS;
    fileinfo.flashtype = MICO_FLASH_FOR_UPDATE;
    strcpy(fileinfo.filename, "mico_ota.bin");

    while((filelen = tget (&fileinfo, ipaddr)) < 0) {
        fota_log("tget return filelen %d, maxretry %d", filelen, maxretry);
        maxretry--;
        if (maxretry < 0) {
            fota_log("ERROR!! Can't get OTA image.");
            free(tmpbuf);
            mico_ota_finished(OTA_NO_FILE, NULL);
            return;
        }
    }

    filelen -= 16; // remove md5.
    fota_log("tftp download image finished, OTA bin len %d", filelen);
    flashaddr = UPDATE_START_ADDRESS + filelen;
    MicoFlashRead(MICO_FLASH_FOR_UPDATE, &flashaddr, (uint8_t *)md5_recv, 16);
    InitMd5( &ctx );
    flashaddr = UPDATE_START_ADDRESS;
    left = filelen;
    while(left > 0) {
        if (left > TMP_BUF_LEN) {
            len = TMP_BUF_LEN;
        } else {
            len = left;
        }
        left -= len;
        MicoFlashRead(MICO_FLASH_FOR_UPDATE, &flashaddr, (uint8_t *)tmpbuf, len);
        Md5Update( &ctx, (uint8_t *)tmpbuf, len);
    }
    Md5Final( &ctx, md5_calc );
    
    if(memcmp(md5_calc, md5_recv, 16) != 0) {
        fota_log("ERROR!! MD5 Error.");
        fota_log("RX:   %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                 md5_recv[0],md5_recv[1],md5_recv[2],md5_recv[3],
                 md5_recv[4],md5_recv[5],md5_recv[6],md5_recv[7],
                 md5_recv[8],md5_recv[9],md5_recv[10],md5_recv[11],
                 md5_recv[12],md5_recv[13],md5_recv[14],md5_recv[15]);
        fota_log("Need: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                 md5_calc[0],md5_calc[1],md5_calc[2],md5_calc[3],
                 md5_calc[4],md5_calc[5],md5_calc[6],md5_calc[7],
                 md5_calc[8],md5_calc[9],md5_calc[10],md5_calc[11],
                 md5_calc[12],md5_calc[13],md5_calc[14],md5_calc[15]);
        mico_ota_finished(OTA_MD5_FAIL, NULL);
        return;
    }

    fota_log("OTA bin md5 check success, upgrading...");

    mico_write_ota_tbl(filelen);

    mico_ota_finished(OTA_SUCCESS, NULL);
    while(1)
        sleep(100);
}


