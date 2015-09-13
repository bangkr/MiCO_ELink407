/**
  ******************************************************************************
  * @file    MICOConfigDelegate.c 
  * @author  William Xu
  * @version V1.0.0
  * @date    05-May-2014
  * @brief   This file provide delegate functons from Easylink function &&
  *          fogcloud data function.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, MXCHIP Inc. SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2014 MXCHIP Inc.</center></h2>
  ******************************************************************************
  */ 

#include "MICO.h"
#include "MICODefine.h"
#include "MICOConfigMenu.h"
#include "JSON-C/json.h"
#include "StringUtils.h"
#include "MicoFogCloud.h"

#ifdef USE_MiCOKit_EXT
#include "micokit_ext.h"   // extension board operation by user.
#endif

#define SYS_LED_TRIGGER_INTERVAL  100 
#define SYS_LED_TRIGGER_INTERVAL_AFTER_EASYLINK  500 
#define RF_LED_TRIGGER_INTERVAL_AFTER_CLOUD_CONNECTED  1000 
#define SYS_LED_TRIGGER_OTA_INTERVAL    50 
  
#define config_delegate_log(M, ...) custom_log("Config Delegate", M, ##__VA_ARGS__)
#define config_delegate_log_trace() custom_log_trace("Config Delegate")

static mico_timer_t _Led_EL_timer;
static void _led_EL_Timeout_handler( void* arg );


void OTAWillStart( mico_Context_t * const inContext )
{
  //config_delegate_log_trace();
  (void)(inContext); 
#ifdef USE_MiCOKit_EXT
  char oled_show_line[OLED_DISPLAY_MAX_CHAR_PER_ROW+1] = {'\0'};
#endif

  /*Led trigger*/
  mico_init_timer(&_Led_EL_timer, SYS_LED_TRIGGER_OTA_INTERVAL, _led_EL_Timeout_handler, NULL);
  mico_start_timer(&_Led_EL_timer);
  
#ifdef USE_MiCOKit_EXT
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%s", (uint8_t*)"Firmware update ");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, (uint8_t*)oled_show_line);
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_3, "  Donwloading...");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_4, "  will reboot...");
#endif
  return;
}

void OTAFailed( mico_Context_t * const inContext )
{
  //config_delegate_log_trace();
  (void)(inContext); 
#ifdef USE_MiCOKit_EXT
  char oled_show_line[OLED_DISPLAY_MAX_CHAR_PER_ROW+1] = {'\0'};
#endif

  // led stop
  mico_stop_timer(&_Led_EL_timer);
  mico_deinit_timer( &_Led_EL_timer );
  MicoSysLed(true);
  
#ifdef USE_MiCOKit_EXT
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%s", (uint8_t*)"OTA...          ");
 //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, (uint8_t*)oled_show_line);
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_3, "  Failed!       ");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_4, "                ");
#endif
  return;
}

void OTASuccess( mico_Context_t * const inContext )
{
  //config_delegate_log_trace();
  (void)(inContext); 
#ifdef USE_MiCOKit_EXT
  char oled_show_line[OLED_DISPLAY_MAX_CHAR_PER_ROW+1] = {'\0'};
#endif
  
  // led stop
  mico_stop_timer(&_Led_EL_timer);
  mico_deinit_timer( &_Led_EL_timer );
  MicoSysLed(true);

#ifdef USE_MiCOKit_EXT
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%s", (uint8_t*)"OTA...          ");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, (uint8_t*)oled_show_line);
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_3, "  Success!      ");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_4, "    reboot...   ");
#endif
  return;
}

void set_RF_LED_cloud_connected( mico_Context_t * const inContext )
{
  config_delegate_log_trace();
  (void)(inContext); 
  /*Led trigger*/
  // TODO: timer cause hardfault
//  mico_init_timer(&_Led_RF_timer, RF_LED_TRIGGER_INTERVAL_AFTER_CLOUD_CONNECTED, _led_RF_Timeout_handler, NULL);
//  mico_start_timer(&_Led_RF_timer);
   MicoGpioOutputTrigger((mico_gpio_t)MICO_SYS_LED);
  
  return;
}
 
void set_RF_LED_cloud_disconnected( mico_Context_t * const inContext )
{
  config_delegate_log_trace();
  // TODO: timer cause hardfault
//  mico_stop_timer(&_Led_RF_timer);
//  mico_deinit_timer( &_Led_RF_timer );
  
  if(inContext->appStatus.isWifiConnected){
    MicoSysLed(true);
  }
  else{
    MicoSysLed(false);
  }
  
  return;
}

static void _led_EL_Timeout_handler( void* arg )
{
  (void)(arg);
  MicoGpioOutputTrigger((mico_gpio_t)MICO_SYS_LED);
}

void ConfigWillStart( mico_Context_t * const inContext )
{
  config_delegate_log_trace();
  (void)(inContext); 
#ifdef USE_MiCOKit_EXT
  char oled_show_line[OLED_DISPLAY_MAX_CHAR_PER_ROW+1] = {'\0'};
#endif
    /*Led trigger*/
  mico_init_timer(&_Led_EL_timer, SYS_LED_TRIGGER_INTERVAL, _led_EL_Timeout_handler, NULL);
  mico_start_timer(&_Led_EL_timer);
#ifdef USE_MiCOKit_EXT
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%s", (uint8_t*)"Config...       ");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, (uint8_t*)oled_show_line);
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_3, "  Awaiting      ");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_4, "    ssid/key    ");
#endif
  
  inContext->appStatus.noOTACheckOnSystemStart = true;
  
  return;
}

void ConfigWillStop( mico_Context_t * const inContext )
{
  (void)(inContext); 
  config_delegate_log_trace();
#ifdef USE_MiCOKit_EXT
  char oled_show_line[OLED_DISPLAY_MAX_CHAR_PER_ROW+1] = {'\0'};
#endif

  mico_stop_timer(&_Led_EL_timer);
  mico_deinit_timer( &_Led_EL_timer );
  MicoGpioOutputLow((mico_gpio_t)MICO_SYS_LED);
#ifdef USE_MiCOKit_EXT
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%s", (uint8_t*)"Config          ");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, (uint8_t*)oled_show_line);
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_3, "    Stop.       ");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_4, "                ");
#endif
  return;
}

void ConfigAirkissIsSuccess( mico_Context_t * const inContext )
{
  (void)(inContext); 
  config_delegate_log_trace();
  #ifdef USE_MiCOKit_EXT
  char oled_show_line[OLED_DISPLAY_MAX_CHAR_PER_ROW+1] = {'\0'};
#endif

  mico_stop_timer(&_Led_EL_timer);
  mico_deinit_timer( &_Led_EL_timer );
  mico_init_timer(&_Led_EL_timer, SYS_LED_TRIGGER_INTERVAL_AFTER_EASYLINK, _led_EL_Timeout_handler, NULL);
  mico_start_timer(&_Led_EL_timer);
#ifdef USE_MiCOKit_EXT
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%s", (uint8_t*)"Airkiss got     ");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, (uint8_t*)oled_show_line);
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%16s", inContext->flashContentInRam.micoSystemConfig.ssid);
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_3, (uint8_t*)oled_show_line);
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%16s", inContext->flashContentInRam.micoSystemConfig.user_key);
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_4, (uint8_t*)oled_show_line);
#endif
  return;
}

void ConfigEasyLinkIsSuccess( mico_Context_t * const inContext )
{
  (void)(inContext); 
  config_delegate_log_trace();
#ifdef USE_MiCOKit_EXT
  char oled_show_line[OLED_DISPLAY_MAX_CHAR_PER_ROW+1] = {'\0'};
#endif

  mico_stop_timer(&_Led_EL_timer);
  mico_deinit_timer( &_Led_EL_timer );
  mico_init_timer(&_Led_EL_timer, SYS_LED_TRIGGER_INTERVAL_AFTER_EASYLINK, _led_EL_Timeout_handler, NULL);
  mico_start_timer(&_Led_EL_timer);
#ifdef USE_MiCOKit_EXT
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%s", (uint8_t*)"EasyLink got    ");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, (uint8_t*)oled_show_line);
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%16s", inContext->flashContentInRam.micoSystemConfig.ssid);
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_3, (uint8_t*)oled_show_line);
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%16s", inContext->flashContentInRam.micoSystemConfig.user_key);
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_4, (uint8_t*)oled_show_line);
#endif
  return;
}

void ConfigSoftApWillStart(mico_Context_t * const inContext )
{
#ifdef USE_MiCOKit_EXT
  char oled_show_line[OLED_DISPLAY_MAX_CHAR_PER_ROW+1] = {'\0'};
#endif
  mico_stop_timer(&_Led_EL_timer);
  mico_deinit_timer( &_Led_EL_timer );
  mico_init_timer(&_Led_EL_timer, SYS_LED_TRIGGER_INTERVAL_AFTER_EASYLINK, _led_EL_Timeout_handler, NULL);
  mico_start_timer(&_Led_EL_timer);
#ifdef USE_MiCOKit_EXT
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%s", "SoftAP...       ");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, (uint8_t*)oled_show_line);
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, " EasyLink_%c%c%c%c%c%c", 
           inContext->micoStatus.mac[9],  inContext->micoStatus.mac[10],
           inContext->micoStatus.mac[12], inContext->micoStatus.mac[13],
           inContext->micoStatus.mac[15], inContext->micoStatus.mac[16] );
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_3, (uint8_t*)oled_show_line);
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_4, "                ");
#endif

  return;
}

OSStatus ConfigELRecvAuthData(char * anthData, mico_Context_t * const inContext )
{
  config_delegate_log_trace();
  (void)(inContext);
  (void)(anthData);
  return kNoErr;
}

json_object* ConfigCreateReportJsonMessage( mico_Context_t * const inContext )
{
  OSStatus err = kNoErr;
  config_delegate_log_trace();
  char name[50], *tempString;
  OTA_Versions_t versions;
  char rfVersion[50];
//  char *rfVer = NULL, *rfVerTemp = NULL;
  json_object *sectors, *sector, *subMenuSectors, *subMenuSector, *mainObject = NULL;
  
#ifdef USE_MiCOKit_EXT
  char oled_show_line[OLED_DISPLAY_MAX_CHAR_PER_ROW+1] = {'\0'};
  char server_ip[16] = {0};

  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%s", (uint8_t*)"FTC...          ");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, (uint8_t*)oled_show_line);
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%16s", inet_ntoa(server_ip,inContext->flashContentInRam.micoSystemConfig.easylinkServerIP));
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_3, (uint8_t*)oled_show_line);
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%16s", (uint8_t*)"Config upload...");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_4, (uint8_t*)oled_show_line);
#endif

  MicoGetRfVer( rfVersion, 50 );

  if(inContext->flashContentInRam.micoSystemConfig.configured == wLanUnConfigured){
    /*You can upload a specific menu*/
  }

  mico_rtos_lock_mutex(&inContext->flashContentInRam_mutex);
  snprintf(name, 50, "%s(%c%c%c%c%c%c)",MODEL, 
                                        inContext->micoStatus.mac[9],  inContext->micoStatus.mac[10], 
                                        inContext->micoStatus.mac[12], inContext->micoStatus.mac[13],
                                        inContext->micoStatus.mac[15], inContext->micoStatus.mac[16]);

  versions.fwVersion = FIRMWARE_REVISION;
  versions.hdVersion = HARDWARE_REVISION;
  versions.protocol =  PROTOCOL;
  versions.rfVersion = NULL;

  sectors = json_object_new_array();
  require( sectors, exit );

  err = MICOAddTopMenu(&mainObject, name, sectors, versions);
  require_noerr(err, exit);

  /*Sector 1*/
  sector = json_object_new_array();
  require( sector, exit );
  err = MICOAddSector(sectors, "MICO SYSTEM",    sector);
  require_noerr(err, exit);

    /*name cell*/
    err = MICOAddStringCellToSector(sector, "Device Name",    inContext->flashContentInRam.micoSystemConfig.name,               "RW", NULL);
    require_noerr(err, exit);

    //Bonjour switcher cell
    err = MICOAddSwitchCellToSector(sector, "Bonjour",        inContext->flashContentInRam.micoSystemConfig.bonjourEnable,      "RW");
    require_noerr(err, exit);

    //RF power save switcher cell
    err = MICOAddSwitchCellToSector(sector, "RF power save",  inContext->flashContentInRam.micoSystemConfig.rfPowerSaveEnable,  "RW");
    require_noerr(err, exit);

    //MCU power save switcher cell
    err = MICOAddSwitchCellToSector(sector, "MCU power save", inContext->flashContentInRam.micoSystemConfig.mcuPowerSaveEnable, "RW");
    require_noerr(err, exit);

    /*sub menu*/
    subMenuSectors = json_object_new_array();
    require( subMenuSectors, exit );
    err = MICOAddMenuCellToSector(sector, "Detail", subMenuSectors);
    require_noerr(err, exit);
      
      subMenuSector = json_object_new_array();
      require( subMenuSector, exit );
      err = MICOAddSector(subMenuSectors,  "",    subMenuSector);
      require_noerr(err, exit);

        err = MICOAddStringCellToSector(subMenuSector, "Firmware Rev.",  FIRMWARE_REVISION, "RO", NULL);
        require_noerr(err, exit);
        err = MICOAddStringCellToSector(subMenuSector, "Hardware Rev.",  HARDWARE_REVISION, "RO", NULL);
        require_noerr(err, exit);
        err = MICOAddStringCellToSector(subMenuSector, "MICO OS Rev.",   MicoGetVer(),      "RO", NULL);
        require_noerr(err, exit);
        err = MICOAddStringCellToSector(subMenuSector, "RF Driver Rev.", rfVersion,         "RO", NULL);
        require_noerr(err, exit);
        err = MICOAddStringCellToSector(subMenuSector, "Model",          MODEL,             "RO", NULL);
        require_noerr(err, exit);
        err = MICOAddStringCellToSector(subMenuSector, "Manufacturer",   MANUFACTURER,      "RO", NULL);
        require_noerr(err, exit);
        err = MICOAddStringCellToSector(subMenuSector, "Protocol",       PROTOCOL,          "RO", NULL);
        require_noerr(err, exit);

      subMenuSector = json_object_new_array();
      err = MICOAddSector(subMenuSectors,  "WLAN",    subMenuSector);
      require_noerr(err, exit);
      
        tempString = DataToHexStringWithColons( (uint8_t *)inContext->flashContentInRam.micoSystemConfig.bssid, 6 );
        err = MICOAddStringCellToSector(subMenuSector, "BSSID",        tempString, "RO", NULL);
        require_noerr(err, exit);
        free(tempString);

        err = MICOAddNumberCellToSector(subMenuSector, "Channel",      inContext->flashContentInRam.micoSystemConfig.channel, "RO", NULL);
        require_noerr(err, exit);

        switch(inContext->flashContentInRam.micoSystemConfig.security){
          case SECURITY_TYPE_NONE:
            err = MICOAddStringCellToSector(subMenuSector, "Security",   "Open system", "RO", NULL); 
            break;
          case SECURITY_TYPE_WEP:
            err = MICOAddStringCellToSector(subMenuSector, "Security",   "WEP",         "RO", NULL); 
            break;
          case SECURITY_TYPE_WPA_TKIP:
            err = MICOAddStringCellToSector(subMenuSector, "Security",   "WPA TKIP",    "RO", NULL); 
            break;
          case SECURITY_TYPE_WPA_AES:
            err = MICOAddStringCellToSector(subMenuSector, "Security",   "WPA AES",     "RO", NULL); 
            break;
          case SECURITY_TYPE_WPA2_TKIP:
            err = MICOAddStringCellToSector(subMenuSector, "Security",   "WPA2 TKIP",   "RO", NULL); 
            break;
          case SECURITY_TYPE_WPA2_AES:
            err = MICOAddStringCellToSector(subMenuSector, "Security",   "WPA2 AES",    "RO", NULL); 
            break;
          case SECURITY_TYPE_WPA2_MIXED:
            err = MICOAddStringCellToSector(subMenuSector, "Security",   "WPA2 MIXED",  "RO", NULL); 
            break;
          default:
            err = MICOAddStringCellToSector(subMenuSector, "Security",   "Auto",      "RO", NULL); 
            break;
        }
        require_noerr(err, exit); 

        if(inContext->flashContentInRam.micoSystemConfig.keyLength == maxKeyLen){ /*This is a PMK key, generated by user key in WPA security type*/
          tempString = calloc(maxKeyLen+1, 1);
          require_action(tempString, exit, err=kNoMemoryErr);
          memcpy(tempString, inContext->flashContentInRam.micoSystemConfig.key, maxKeyLen);
          err = MICOAddStringCellToSector(subMenuSector, "PMK",          tempString, "RO", NULL);
          require_noerr(err, exit);
          free(tempString);
        }
        else{
          err = MICOAddStringCellToSector(subMenuSector, "KEY",          inContext->flashContentInRam.micoSystemConfig.user_key,  "RO", NULL);
          require_noerr(err, exit);
        }

  /*Sector 3*/
  sector = json_object_new_array();
  require( sector, exit );
  err = MICOAddSector(sectors, "WLAN",           sector);
  require_noerr(err, exit);
    /*SSID cell*/
    err = MICOAddStringCellToSector(sector, "Wi-Fi",        inContext->flashContentInRam.micoSystemConfig.ssid,     "RW", NULL);
    require_noerr(err, exit);
    /*PASSWORD cell*/
    err = MICOAddStringCellToSector(sector, "Password",     inContext->flashContentInRam.micoSystemConfig.user_key, "RW", NULL);
    require_noerr(err, exit);
    /*DHCP cell*/
    err = MICOAddSwitchCellToSector(sector, "DHCP",        inContext->flashContentInRam.micoSystemConfig.dhcpEnable,   "RW");
    require_noerr(err, exit);
    /*Local cell*/
    err = MICOAddStringCellToSector(sector, "IP address",  inContext->micoStatus.localIp,   "RW", NULL);
    require_noerr(err, exit);
    /*Netmask cell*/
    err = MICOAddStringCellToSector(sector, "Net Mask",    inContext->micoStatus.netMask,   "RW", NULL);
    require_noerr(err, exit);
    /*Gateway cell*/
    err = MICOAddStringCellToSector(sector, "Gateway",     inContext->micoStatus.gateWay,   "RW", NULL);
    require_noerr(err, exit);
    /*DNS server cell*/
    err = MICOAddStringCellToSector(sector, "DNS Server",  inContext->micoStatus.dnsServer, "RW", NULL);
    require_noerr(err, exit);

  /*Sector 4*/

  /*Sector 5*/
  sector = json_object_new_array();
  require( sector, exit );
  err = MICOAddSector(sectors, "MCU IOs",            sector);
  require_noerr(err, exit);

    /*UART Baurdrate cell*/
    json_object *selectArray;
    selectArray = json_object_new_array();
    require( selectArray, exit );
    json_object_array_add(selectArray, json_object_new_int(9600));
    json_object_array_add(selectArray, json_object_new_int(19200));
    json_object_array_add(selectArray, json_object_new_int(38400));
    json_object_array_add(selectArray, json_object_new_int(57600));
    json_object_array_add(selectArray, json_object_new_int(115200));
    err = MICOAddNumberCellToSector(sector, "Baurdrate", 115200, "RW", selectArray);
    //err = MICOAddNumberCellToSector(sector, "Baurdrate", 
    //          inContext->flashContentInRam.appConfig.fogcloudConfig.USART_BaudRate, 
    //          "RW", selectArray);
    require_noerr(err, exit);
    
  /*Sector 6: cloud settings*/
  sector = json_object_new_array();
  require( sector, exit );
  err = MICOAddSector(sectors, "Cloud info", sector);
  require_noerr(err, exit);
  
  // device activate status
  err = MICOAddSwitchCellToSector(sector, "activated", 
                                  inContext->flashContentInRam.appConfig.fogcloudConfig.isActivated, 
                                  "RO");
  // device binding status
  err = MICOAddSwitchCellToSector(sector, "binding", 
                                  inContext->flashContentInRam.appConfig.fogcloudConfig.owner_binding, 
                                  "RO");
  require_noerr(err, exit);
  // cloud connect status
  err = MICOAddSwitchCellToSector(sector, "connected", 
                                  inContext->appStatus.fogcloudStatus.isCloudConnected, 
                                  "RO");
  require_noerr(err, exit);
  // rom version cell
  err = MICOAddStringCellToSector(sector, "rom version", 
                                  inContext->flashContentInRam.appConfig.fogcloudConfig.romVersion,
                                  "RO", NULL);
  require_noerr(err, exit);
  // device_id cell, is RO in fact, we set RW is convenient for read full string.
  err = MICOAddStringCellToSector(sector, "device_id", 
                                  inContext->flashContentInRam.appConfig.fogcloudConfig.deviceId,
                                  "RW", NULL);
  /*sub menu - cloud setting */
  subMenuSectors = json_object_new_array();
  require( subMenuSectors, exit );
  err = MICOAddMenuCellToSector(sector, "Cloud settings", subMenuSectors);
  require_noerr(err, exit);
  
  subMenuSector = json_object_new_array();
  require( subMenuSector, exit );
  err = MICOAddSector(subMenuSectors, "Authentication", subMenuSector);
  require_noerr(err, exit);
  
  err = MICOAddStringCellToSector(subMenuSector, "loginId",  
                                  inContext->flashContentInRam.appConfig.fogcloudConfig.loginId,
                                  "RW", NULL);
  err = MICOAddStringCellToSector(subMenuSector, "devPasswd",  
                                  inContext->flashContentInRam.appConfig.fogcloudConfig.devPasswd,
                                  "RW", NULL);
  err = MICOAddStringCellToSector(subMenuSector, "userToken",  
                                  inContext->flashContentInRam.appConfig.fogcloudConfig.userToken,
                                  "RW", NULL);
  mico_rtos_unlock_mutex(&inContext->flashContentInRam_mutex);
  
exit:
  if(err != kNoErr && mainObject){
    json_object_put(mainObject);
    mainObject = NULL;
  }
  return mainObject;
}

OSStatus ConfigIncommingJsonMessage( const char *input, mico_Context_t * const inContext )
{
  OSStatus err = kNoErr;
  json_object *new_obj;
  config_delegate_log_trace();
  
#ifdef USE_MiCOKit_EXT
  char oled_show_line[OLED_DISPLAY_MAX_CHAR_PER_ROW+1] = {'\0'};
  char server_ip[16] = {0};
  
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%s", (uint8_t*)"FTC...          ");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, (uint8_t*)oled_show_line);
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%16s", inet_ntoa(server_ip,inContext->flashContentInRam.micoSystemConfig.easylinkServerIP));
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_3, (uint8_t*)oled_show_line);
  memset(oled_show_line, '\0', OLED_DISPLAY_MAX_CHAR_PER_ROW+1);
  snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW+1, "%16s", (uint8_t*)"Config set...");
  //OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_4, (uint8_t*)oled_show_line);
#endif

  new_obj = json_tokener_parse(input);
  require_action(new_obj, exit, err = kUnknownErr);
  config_delegate_log("Recv config object=%s", json_object_to_json_string(new_obj));
  mico_rtos_lock_mutex(&inContext->flashContentInRam_mutex);
  json_object_object_foreach(new_obj, key, val) {
    if(!strcmp(key, "Device Name")){
      strncpy(inContext->flashContentInRam.micoSystemConfig.name, json_object_get_string(val), maxNameLen);
    }else if(!strcmp(key, "RF power save")){
      inContext->flashContentInRam.micoSystemConfig.rfPowerSaveEnable = json_object_get_boolean(val);
    }else if(!strcmp(key, "MCU power save")){
      inContext->flashContentInRam.micoSystemConfig.mcuPowerSaveEnable = json_object_get_boolean(val);
    }else if(!strcmp(key, "Bonjour")){
      inContext->flashContentInRam.micoSystemConfig.bonjourEnable = json_object_get_boolean(val);
    }else if(!strcmp(key, "Wi-Fi")){
      strncpy(inContext->flashContentInRam.micoSystemConfig.ssid, json_object_get_string(val), maxSsidLen);
      inContext->flashContentInRam.micoSystemConfig.channel = 0;
      memset(inContext->flashContentInRam.micoSystemConfig.bssid, 0x0, 6);
      inContext->flashContentInRam.micoSystemConfig.security = SECURITY_TYPE_AUTO;
      memcpy(inContext->flashContentInRam.micoSystemConfig.key, inContext->flashContentInRam.micoSystemConfig.user_key, maxKeyLen);
      inContext->flashContentInRam.micoSystemConfig.keyLength = inContext->flashContentInRam.micoSystemConfig.user_keyLength;
    }else if(!strcmp(key, "Password")){
      inContext->flashContentInRam.micoSystemConfig.security = SECURITY_TYPE_AUTO;
      strncpy(inContext->flashContentInRam.micoSystemConfig.key, json_object_get_string(val), maxKeyLen);
      strncpy(inContext->flashContentInRam.micoSystemConfig.user_key, json_object_get_string(val), maxKeyLen);
      inContext->flashContentInRam.micoSystemConfig.keyLength = strlen(inContext->flashContentInRam.micoSystemConfig.key);
      inContext->flashContentInRam.micoSystemConfig.user_keyLength = strlen(inContext->flashContentInRam.micoSystemConfig.key);
    }else if(!strcmp(key, "DHCP")){
      inContext->flashContentInRam.micoSystemConfig.dhcpEnable   = json_object_get_boolean(val);
    }else if(!strcmp(key, "IP address")){
      strncpy(inContext->flashContentInRam.micoSystemConfig.localIp, json_object_get_string(val), maxIpLen);
    }else if(!strcmp(key, "Net Mask")){
      strncpy(inContext->flashContentInRam.micoSystemConfig.netMask, json_object_get_string(val), maxIpLen);
    }else if(!strcmp(key, "Gateway")){
      strncpy(inContext->flashContentInRam.micoSystemConfig.gateWay, json_object_get_string(val), maxIpLen);
    }else if(!strcmp(key, "DNS Server")){
      strncpy(inContext->flashContentInRam.micoSystemConfig.dnsServer, json_object_get_string(val), maxIpLen);   
    }/*else if(!strcmp(key, "Baurdrate")){
      inContext->flashContentInRam.appConfig.fogcloudConfig.USART_BaudRate = json_object_get_int(val);
    } */else if(!strcmp(key, "loginId")){
      strncpy(inContext->flashContentInRam.appConfig.fogcloudConfig.loginId, json_object_get_string(val), MAX_SIZE_LOGIN_ID); 
    } else if(!strcmp(key, "devpasswd")){
      strncpy(inContext->flashContentInRam.appConfig.fogcloudConfig.devPasswd, json_object_get_string(val), MAX_SIZE_DEV_PASSWD); 
    } else if(!strcmp(key, "userToken")){
      strncpy(inContext->flashContentInRam.appConfig.fogcloudConfig.userToken, json_object_get_string(val), MAX_SIZE_USER_TOKEN); 
    }else{
    }
  }
  json_object_put(new_obj);
  mico_rtos_unlock_mutex(&inContext->flashContentInRam_mutex);

exit:
  return err; 
}

/*******************************************************************************
 * get json data of fogcloud local tcp request 
 * for device state/activate/authorize/reset/ota
 ******************************************************************************/
OSStatus getMVDActivateRequestData(const char *input, MVDActivateRequestData_t *activateData)
{
  OSStatus err = kUnknownErr;
  json_object *new_obj;
  config_delegate_log_trace();
  new_obj = json_tokener_parse(input);
  require_action(new_obj, exit, err = kUnknownErr);
  config_delegate_log("Recv activate object=%s", json_object_to_json_string(new_obj));
  json_object_object_foreach(new_obj, key, val) {
    if(!strcmp(key, "login_id")){
      strncpy(activateData->loginId, json_object_get_string(val), MAX_SIZE_LOGIN_ID);
    }
    else if(!strcmp(key, "dev_passwd")){
      strncpy(activateData->devPasswd, json_object_get_string(val), MAX_SIZE_DEV_PASSWD);
    }
    else if(!strcmp(key, "user_token")){
      strncpy(activateData->user_token, json_object_get_string(val), MAX_SIZE_USER_TOKEN);
    }
  }
  json_object_put(new_obj);
  err = kNoErr;
exit:
  return err;
}

OSStatus getMVDAuthorizeRequestData(const char *input, MVDAuthorizeRequestData_t *authorizeData)
{
  OSStatus err = kUnknownErr;
  json_object *new_obj;
  config_delegate_log_trace();
  new_obj = json_tokener_parse(input);
  require_action(new_obj, exit, err = kUnknownErr);
  config_delegate_log("Recv authorize object=%s", json_object_to_json_string(new_obj));
  json_object_object_foreach(new_obj, key, val) {
    if(!strcmp(key, "login_id")){
      strncpy(authorizeData->loginId, json_object_get_string(val), MAX_SIZE_LOGIN_ID);
    }
    else if(!strcmp(key, "dev_passwd")){
      strncpy(authorizeData->devPasswd, json_object_get_string(val), MAX_SIZE_DEV_PASSWD);
    }
    else if(!strcmp(key, "user_token")){
      strncpy(authorizeData->user_token, json_object_get_string(val), MAX_SIZE_USER_TOKEN);
    }
  }
  json_object_put(new_obj);
  err = kNoErr;
exit:
  return err;
}

OSStatus getMVDResetRequestData(const char *input, MVDResetRequestData_t *devResetData)
{
  OSStatus err = kUnknownErr;
  json_object *new_obj;
  config_delegate_log_trace();
  new_obj = json_tokener_parse(input);
  require_action(new_obj, exit, err = kUnknownErr);
  config_delegate_log("Recv devReset object=%s", json_object_to_json_string(new_obj));
  json_object_object_foreach(new_obj, key, val) {
    if(!strcmp(key, "login_id")){
      strncpy(devResetData->loginId, json_object_get_string(val), MAX_SIZE_LOGIN_ID);
    }
    else if(!strcmp(key, "dev_passwd")){
      strncpy(devResetData->devPasswd, json_object_get_string(val), MAX_SIZE_DEV_PASSWD);
    }
    else {
    }
  }
  json_object_put(new_obj);
  err = kNoErr;
exit:
  return err;
}

OSStatus getMVDOTARequestData(const char *input, MVDOTARequestData_t *OTAData)
{
  OSStatus err = kUnknownErr;
  json_object *new_obj;
  config_delegate_log_trace();
  new_obj = json_tokener_parse(input);
  require_action(new_obj, exit, err = kUnknownErr);
  config_delegate_log("Recv OTA request object=%s", json_object_to_json_string(new_obj));
  json_object_object_foreach(new_obj, key, val) {
    if(!strcmp(key, "login_id")){
      strncpy(OTAData->loginId, json_object_get_string(val), MAX_SIZE_LOGIN_ID);
    }
    else if(!strcmp(key, "dev_passwd")){
      strncpy(OTAData->devPasswd, json_object_get_string(val), MAX_SIZE_DEV_PASSWD);
    }
    else {
    }
  }
  json_object_put(new_obj);
  err = kNoErr;
exit:
  return err;
}

OSStatus getMVDGetStateRequestData(const char *input, MVDGetStateRequestData_t *devGetStateData)
{
  OSStatus err = kUnknownErr;
  json_object *new_obj;
  config_delegate_log_trace();
  new_obj = json_tokener_parse(input);
  require_action(new_obj, exit, err = kUnknownErr);
  config_delegate_log("Recv activate object=%s", json_object_to_json_string(new_obj));
  json_object_object_foreach(new_obj, key, val) {
    if(!strcmp(key, "login_id")){
      strncpy(devGetStateData->loginId, json_object_get_string(val), MAX_SIZE_LOGIN_ID);
    }
    else if(!strcmp(key, "dev_passwd")){
      strncpy(devGetStateData->devPasswd, json_object_get_string(val), MAX_SIZE_DEV_PASSWD);
    }
    else if(!strcmp(key, "user_token")){
      strncpy(devGetStateData->user_token, json_object_get_string(val), MAX_SIZE_USER_TOKEN);
    }
  }
  json_object_put(new_obj);
  err = kNoErr;
exit:
  return err;
}
