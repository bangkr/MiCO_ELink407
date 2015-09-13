/**
******************************************************************************
* @file    wifi_config.c 
* @author  William Xu
* @version V1.0.0
* @date    05-May-2014
* @brief   This file provide the easylink && airkiss function.
******************************************************************************
*
*  The MIT License
*  Copyright (c) 2014 MXCHIP Inc.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy 
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights 
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is furnished
*  to do so, subject to the following conditions:
*
*  The above copyright notice and this permission notice shall be included in
*  all copies or substantial portions of the Software.
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR 
*  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
******************************************************************************
*/

#include "MICO.h"
#include "MICONotificationCenter.h"

#include "MicoPlatform.h"
#include "StringUtils.h"
#include "HTTPUtils.h"
#include "SocketUtils.h"
#include "wifi_config.h"

#define wifi_config_log(M, ...) custom_log("WiFiConfig", M, ##__VA_ARGS__)
#define wifi_config_log_trace() custom_log_trace("WiFiConfig")

typedef enum {
  WIFI_CONFIG_UNKNOWN = 0,
  WIFI_CONFIG_EASYLINK,
  WIFI_CONFIG_AIRKISS
}wifi_config_type_t;

static mico_semaphore_t      wificonfig_sem;
static mico_semaphore_t      config_finished_sem;
static bool                  wificonfig_thread_should_exit = false;
static uint8_t               airkiss_data;
static wifi_config_type_t    get_wifi_config_type = WIFI_CONFIG_UNKNOWN;

static void easylink_thread(void *inContext);
extern void         ConfigWillStart               ( mico_Context_t * const inContext );
extern void         ConfigWillStop                ( mico_Context_t * const inContext );
extern void         ConfigEasyLinkIsSuccess       ( mico_Context_t * const inContext );
extern void         ConfigAirkissIsSuccess        ( mico_Context_t * const inContext );
extern OSStatus     ConfigELRecvAuthData          ( char * userInfo, mico_Context_t * const inContext );


void EasyLinkNotify_SYSWillPowerOffHandler(mico_Context_t * const inContext)
{
  stopWifiConfig(inContext);
}

void EasyLinkNotify_WifiStatusHandler(WiFiEvent event, mico_Context_t * const inContext)
{
  wifi_config_log_trace();
  IPStatusTypedef para;

  require(inContext, exit);
  switch (event) {
  case NOTIFY_STATION_UP:
    MicoRfLed(true);
    wifi_config_log("Access point connected");
    mico_rtos_set_semaphore(&wificonfig_sem);
    micoWlanGetIPStatus(&para, Station);
    strncpy(inContext->flashContentInRam.micoSystemConfig.localIp, para.ip, maxIpLen);
    strncpy(inContext->flashContentInRam.micoSystemConfig.netMask, para.mask, maxIpLen);
    strncpy(inContext->flashContentInRam.micoSystemConfig.gateWay, para.gate, maxIpLen);
    strncpy(inContext->flashContentInRam.micoSystemConfig.dnsServer, para.dns, maxIpLen);
    break;
  case NOTIFY_STATION_DOWN:
    MicoRfLed(false);
    break;
  default:
    break;
  }
  
exit:
  return;
}

void EasyLinkNotify_EasyLinkButtonClickedHandler(mico_Context_t * const inContext)
{
  (void)inContext;
}


void EasyLinkNotify_WiFIParaChangedHandler(apinfo_adv_t *ap_info, char *key, int key_len, mico_Context_t * const inContext)
{
  wifi_config_log_trace();
  require(inContext, exit);
  mico_rtos_lock_mutex(&inContext->flashContentInRam_mutex);
  memcpy(inContext->flashContentInRam.micoSystemConfig.ssid, ap_info->ssid, maxSsidLen);
  memcpy(inContext->flashContentInRam.micoSystemConfig.bssid, ap_info->bssid, 6);
  inContext->flashContentInRam.micoSystemConfig.channel = ap_info->channel;
  inContext->flashContentInRam.micoSystemConfig.security = ap_info->security;
  memcpy(inContext->flashContentInRam.micoSystemConfig.key, key, maxKeyLen);
  inContext->flashContentInRam.micoSystemConfig.keyLength = key_len;
  mico_rtos_unlock_mutex(&inContext->flashContentInRam_mutex);
exit:
  return;
}

void EasyLinkNotify_DHCPCompleteHandler(IPStatusTypedef *pnet, mico_Context_t * const inContext)
{
  wifi_config_log_trace();
  require(inContext, exit);
  strcpy((char *)inContext->micoStatus.localIp, pnet->ip);
  strcpy((char *)inContext->micoStatus.netMask, pnet->mask);
  strcpy((char *)inContext->micoStatus.gateWay, pnet->gate);
  strcpy((char *)inContext->micoStatus.dnsServer, pnet->dns);
exit:
  return;
}

void EasyLinkNotify_EasyLinkCompleteHandler(network_InitTypeDef_st *nwkpara, mico_Context_t * const inContext)
{
  OSStatus err;
  wifi_config_log_trace();
  wifi_config_log("EasyLink return");
  require_action(inContext, exit, err = kParamErr);
  require_action(nwkpara, exit, err = kTimeoutErr);
  
  mico_rtos_lock_mutex(&inContext->flashContentInRam_mutex);
  memcpy(inContext->flashContentInRam.micoSystemConfig.ssid, nwkpara->wifi_ssid, maxSsidLen);
  memset(inContext->flashContentInRam.micoSystemConfig.bssid, 0x0, 6);
  memcpy(inContext->flashContentInRam.micoSystemConfig.user_key, nwkpara->wifi_key, maxKeyLen);
  inContext->flashContentInRam.micoSystemConfig.user_keyLength = strlen(nwkpara->wifi_key);
  inContext->flashContentInRam.micoSystemConfig.dhcpEnable = true;
  mico_rtos_unlock_mutex(&inContext->flashContentInRam_mutex);
  wifi_config_log("Get SSID: %s, Key: %s", inContext->flashContentInRam.micoSystemConfig.ssid, inContext->flashContentInRam.micoSystemConfig.user_key);
  return;

/*EasyLink timeout or error*/    
exit:
  wifi_config_log("ERROR, err: %d", err);
  wificonfig_thread_should_exit = true;
  mico_rtos_set_semaphore(&wificonfig_sem);

  return;
}

// Data = AuthData#FTCServer(localIp/netMask/gateWay/dnsServer)
void EasyLinkNotify_EasyLinkGetExtraDataHandler(int datalen, char* data, mico_Context_t * const inContext)
{
  OSStatus err;
  int index ;
  char address[16];
  wifi_config_log_trace();
  uint32_t *ipInfo, ipInfoCount;
  require_action(inContext, exit, err = kParamErr);
  char *debugString;

  debugString = DataToHexStringWithSpaces( (const uint8_t *)data, datalen );
  wifi_config_log("Get user info: %s", debugString);
  free(debugString);
  
  if(1 == datalen){  // extra data from Airkiss, datalen == 1 byte  
    wifi_config_log("Wi-Fi configured by Airkiss.");
    airkiss_data = data[0];
    mico_rtos_set_semaphore(&wificonfig_sem);
    ConfigAirkissIsSuccess(inContext);
    get_wifi_config_type = WIFI_CONFIG_AIRKISS;
  }
  else if(datalen > 1){  // extra data from EasyLink, datalen > 1 byte  
    wifi_config_log("Wi-Fi configured by EasyLink.");
    for(index = datalen - 1; index>=0; index-- ){
      if(data[index] == '#' &&( (datalen - index) == 5 || (datalen - index) == 25 ) )
        break;
    }
    require_action(index >= 0, exit, err = kParamErr);
    
    data[index++] = 0x0;
    ipInfo = (uint32_t *)&data[index];
    ipInfoCount = (datalen - index)/sizeof(uint32_t);
    require_action(ipInfoCount >= 1, exit, err = kParamErr);
    mico_rtos_lock_mutex(&inContext->flashContentInRam_mutex);
    inContext->flashContentInRam.micoSystemConfig.easylinkServerIP = *(uint32_t *)(ipInfo);
    
    if(ipInfoCount == 1){
      inContext->flashContentInRam.micoSystemConfig.dhcpEnable = true;
      inet_ntoa( address, inContext->flashContentInRam.micoSystemConfig.easylinkServerIP);
      wifi_config_log("Get auth info: %s, EasyLink server ip address: %s", data, address);
    }else{
      inContext->flashContentInRam.micoSystemConfig.dhcpEnable = false;
      ipInfo = (uint32_t *)&data[index];
      inet_ntoa(inContext->flashContentInRam.micoSystemConfig.localIp, *(uint32_t *)(ipInfo+1));
      inet_ntoa(inContext->flashContentInRam.micoSystemConfig.netMask, *(uint32_t *)(ipInfo+2));
      inet_ntoa(inContext->flashContentInRam.micoSystemConfig.gateWay, *(uint32_t *)(ipInfo+3));
      inet_ntoa(inContext->flashContentInRam.micoSystemConfig.dnsServer, *(uint32_t *)(ipInfo+4));
      strcpy((char *)inContext->micoStatus.localIp, inContext->flashContentInRam.micoSystemConfig.localIp);
      strcpy((char *)inContext->micoStatus.netMask, inContext->flashContentInRam.micoSystemConfig.netMask);
      strcpy((char *)inContext->micoStatus.gateWay, inContext->flashContentInRam.micoSystemConfig.gateWay);
      strcpy((char *)inContext->micoStatus.dnsServer, inContext->flashContentInRam.micoSystemConfig.dnsServer);
      inet_ntoa( address, inContext->flashContentInRam.micoSystemConfig.easylinkServerIP);
      wifi_config_log("Get auth info: %s, EasyLink server ip address: %s, local IP info:%s %s %s %s ", data, address, inContext->flashContentInRam.micoSystemConfig.localIp,\
        inContext->flashContentInRam.micoSystemConfig.netMask, inContext->flashContentInRam.micoSystemConfig.gateWay,inContext->flashContentInRam.micoSystemConfig.dnsServer);
    }
    mico_rtos_unlock_mutex(&inContext->flashContentInRam_mutex);
    
    require_noerr(ConfigELRecvAuthData(data, inContext), exit);
    mico_rtos_set_semaphore(&wificonfig_sem);
    ConfigEasyLinkIsSuccess(inContext);
    get_wifi_config_type = WIFI_CONFIG_EASYLINK;
  }
  else{
    wifi_config_log("ERROR: Get extra data len = 0.");
    err = kParamErr;
    goto exit;
  }
  
  return;

exit:
  wifi_config_log("ERROR, err: %d", err);
  ConfigWillStop(inContext);
  MicoSystemReboot();
  return;
}

OSStatus startWifiConfig( mico_Context_t * const inContext)
{
  wifi_config_log_trace();
  OSStatus err = kUnknownErr;

  err = MICOAddNotification( mico_notify_WIFI_STATUS_CHANGED, (void *)EasyLinkNotify_WifiStatusHandler );
  require_noerr(err, exit);
  err = MICOAddNotification( mico_notify_WiFI_PARA_CHANGED, (void *)EasyLinkNotify_WiFIParaChangedHandler );
  require_noerr(err, exit);
  err = MICOAddNotification( mico_notify_EASYLINK_WPS_COMPLETED, (void *)EasyLinkNotify_EasyLinkCompleteHandler );
  require_noerr(err, exit);
  err = MICOAddNotification( mico_notify_EASYLINK_GET_EXTRA_DATA, (void *)EasyLinkNotify_EasyLinkGetExtraDataHandler );
  require_noerr(err, exit);
  err = MICOAddNotification( mico_notify_DHCP_COMPLETED, (void *)EasyLinkNotify_DHCPCompleteHandler );
  require_noerr( err, exit );
  err = MICOAddNotification( mico_notify_SYS_WILL_POWER_OFF, (void *)EasyLinkNotify_SYSWillPowerOffHandler );
  require_noerr( err, exit );
  
  // Start the EasyLink thread
  ConfigWillStart(inContext);
  mico_rtos_init_semaphore(&wificonfig_sem, 1);
  mico_rtos_init_semaphore(&config_finished_sem, 1);

  err = mico_rtos_create_thread(NULL, MICO_APPLICATION_PRIORITY, "EASYLINK", easylink_thread, 0x1000, (void*)inContext );
  require_noerr_action( err, exit, wifi_config_log("ERROR: Unable to start the EasyLink thread.") );

  mico_rtos_get_semaphore( &config_finished_sem, MICO_WAIT_FOREVER ); 
  mico_rtos_deinit_semaphore( &config_finished_sem );

exit:
  return err;
}

void connect_wifi_normal( mico_Context_t * const inContext)
{
  wifi_config_log_trace();
  network_InitTypeDef_adv_st wNetConfig;
  memset(&wNetConfig, 0x0, sizeof(network_InitTypeDef_adv_st));
  
  mico_rtos_lock_mutex(&inContext->flashContentInRam_mutex);
  strncpy((char*)wNetConfig.ap_info.ssid, inContext->flashContentInRam.micoSystemConfig.ssid, maxSsidLen);
  wNetConfig.ap_info.security = SECURITY_TYPE_AUTO;
  memcpy(wNetConfig.key, inContext->flashContentInRam.micoSystemConfig.user_key, maxKeyLen);
  wNetConfig.key_len = inContext->flashContentInRam.micoSystemConfig.user_keyLength;
  wNetConfig.dhcpMode = inContext->flashContentInRam.micoSystemConfig.dhcpEnable;
  strncpy((char*)wNetConfig.local_ip_addr, inContext->flashContentInRam.micoSystemConfig.localIp, maxIpLen);
  strncpy((char*)wNetConfig.net_mask, inContext->flashContentInRam.micoSystemConfig.netMask, maxIpLen);
  strncpy((char*)wNetConfig.gateway_ip_addr, inContext->flashContentInRam.micoSystemConfig.gateWay, maxIpLen);
  strncpy((char*)wNetConfig.dnsServer_ip_addr, inContext->flashContentInRam.micoSystemConfig.dnsServer, maxIpLen);

  wNetConfig.wifi_retry_interval = 100;
  mico_rtos_unlock_mutex(&inContext->flashContentInRam_mutex);
  micoWlanStartAdv(&wNetConfig);
  wifi_config_log("connect to %s.....", wNetConfig.ap_info.ssid);
}

void connect_wifi_fast( mico_Context_t * const inContext)
{
  wifi_config_log_trace();
  network_InitTypeDef_adv_st wNetConfig;
  memset(&wNetConfig, 0x0, sizeof(network_InitTypeDef_adv_st));

  mico_rtos_lock_mutex(&inContext->flashContentInRam_mutex);
  strncpy((char*)wNetConfig.ap_info.ssid, inContext->flashContentInRam.micoSystemConfig.ssid, maxSsidLen);
  memcpy(wNetConfig.ap_info.bssid, inContext->flashContentInRam.micoSystemConfig.bssid, 6);
  wNetConfig.ap_info.channel = inContext->flashContentInRam.micoSystemConfig.channel;
  wNetConfig.ap_info.security = inContext->flashContentInRam.micoSystemConfig.security;
  memcpy(wNetConfig.key, inContext->flashContentInRam.micoSystemConfig.key, maxKeyLen);
  wNetConfig.key_len = inContext->flashContentInRam.micoSystemConfig.keyLength;
  wNetConfig.dhcpMode = inContext->flashContentInRam.micoSystemConfig.dhcpEnable;
  strncpy((char*)wNetConfig.local_ip_addr, inContext->flashContentInRam.micoSystemConfig.localIp, maxIpLen);
  strncpy((char*)wNetConfig.net_mask, inContext->flashContentInRam.micoSystemConfig.netMask, maxIpLen);
  strncpy((char*)wNetConfig.gateway_ip_addr, inContext->flashContentInRam.micoSystemConfig.gateWay, maxIpLen);
  strncpy((char*)wNetConfig.dnsServer_ip_addr, inContext->flashContentInRam.micoSystemConfig.dnsServer, maxIpLen);

  wNetConfig.wifi_retry_interval = 100;
  mico_rtos_unlock_mutex(&inContext->flashContentInRam_mutex);
  micoWlanStartAdv(&wNetConfig);
  wifi_config_log("Connect to %s.....\r\n", wNetConfig.ap_info.ssid);
}

void _cleanEasyLinkResource( mico_Context_t * const inContext )
{
  (void)inContext;
  
  /*module should power down under default setting*/ 
  MICORemoveNotification( mico_notify_WIFI_STATUS_CHANGED, (void *)EasyLinkNotify_WifiStatusHandler );
  MICORemoveNotification( mico_notify_WiFI_PARA_CHANGED, (void *)EasyLinkNotify_WiFIParaChangedHandler );
  MICORemoveNotification( mico_notify_EASYLINK_WPS_COMPLETED, (void *)EasyLinkNotify_EasyLinkCompleteHandler );
  MICORemoveNotification( mico_notify_EASYLINK_GET_EXTRA_DATA, (void *)EasyLinkNotify_EasyLinkGetExtraDataHandler );
  MICORemoveNotification( mico_notify_DHCP_COMPLETED, (void *)EasyLinkNotify_DHCPCompleteHandler );
  MICORemoveNotification( mico_notify_SYS_WILL_POWER_OFF, (void *)EasyLinkNotify_SYSWillPowerOffHandler );

  mico_rtos_deinit_semaphore(&wificonfig_sem);
  wificonfig_sem = NULL;
}

OSStatus stopWifiConfig( mico_Context_t * const inContext)
{
  (void)inContext;
  micoWlanStopEasyLink();
  return kNoErr;
}

void easylink_thread(void *inContext)
{
  OSStatus err = kNoErr;
  mico_Context_t *Context = inContext;
  int fd;
  struct sockaddr_t addr;
  int i = 0;

  wifi_config_log_trace();
  require_action(wificonfig_sem, threadexit, err = kNotPreparedErr);
      
  if(Context->flashContentInRam.micoSystemConfig.easyLinkByPass == EASYLINK_BYPASS){
    Context->flashContentInRam.micoSystemConfig.easyLinkByPass = EASYLINK_BYPASS_NO;
    MICOUpdateConfiguration(Context);
    connect_wifi_fast(Context);
  }else{
    wifi_config_log("Start EasyLink(Airkiss)");
    micoWlanStartEasyLink(WifiConfig_TimeOut/1000);
    mico_rtos_get_semaphore(&wificonfig_sem, MICO_WAIT_FOREVER);
  
    // get ssid/key timeout, config thread should exit
    if( wificonfig_thread_should_exit == true ){
      /*so roll back to previous settings  (if it has) and connect*/
      if(Context->flashContentInRam.micoSystemConfig.configured != unConfigured){
        wifi_config_log("WARNING: Timeout, roll back to previous settings...");
        Context->flashContentInRam.micoSystemConfig.configured = allConfigured;
        MICOUpdateConfiguration(Context);
        connect_wifi_normal( Context );
      }else{
        goto reboot;  // reboot && start EasyLink again
      }
      goto threadexit;
    }else{
      connect_wifi_normal( Context );  // get ssid/key, try to connect ap
    }
  }

  // wait for ap connecting, or reboot
  err = mico_rtos_get_semaphore(&wificonfig_sem, WifiConfig_ConnectWlan_Timeout);
  require_noerr(err, reboot); 
  
  // config success, arikiss udp response
  if( WIFI_CONFIG_AIRKISS == get_wifi_config_type){
    fd = socket( AF_INET, SOCK_DGRM, IPPROTO_UDP );
    if (fd < 0)
      goto threadexit;
    
    addr.s_ip = INADDR_BROADCAST;
    addr.s_port = 10000;
    wifi_config_log("Send UDP to WECHAT");
    while(1){
      sendto(fd, &airkiss_data, 1, 0, &addr, sizeof(addr));
      
      msleep(10);
      i++;
      if (i > 20)
        break;
    }
    close(fd);
  }
  Context->flashContentInRam.micoSystemConfig.configured = allConfigured;
  MICOUpdateConfiguration( Context );

threadexit:
  ConfigWillStop( Context );
  micoWlanStopEasyLink();
  _cleanEasyLinkResource( Context );
  mico_rtos_set_semaphore( &config_finished_sem ); 
  mico_rtos_delete_thread( NULL );
  return;
  
/*SSID or Password is not correct, module cannot connect to wlan, so reboot and enter EasyLink again*/
reboot:
  wifi_config_log("Wi-Fi config failed, system reboot...");
  MicoSystemReboot();
  return;
}
