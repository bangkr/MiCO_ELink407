/**
******************************************************************************
* @file    MicoFogCloud.c 
* @author  Eshen Wang
* @version V1.0.0
* @date    17-Mar-2015
* @brief   This file contains the implementations of cloud service interfaces 
*          for MICO.
operation
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

#include "MICODefine.h"
#include "MICONotificationCenter.h"

#include "MicoFogCloud.h"
#include "fogcloud.h"
#include "FogCloudUtils.h"

#define fogcloud_log(M, ...) custom_log("MicoFogCloud", M, ##__VA_ARGS__)
#define fogcloud_log_trace() custom_log_trace("MicoFogCloud")


/*******************************************************************************
 *                                  DEFINES
 ******************************************************************************/


/*******************************************************************************
 *                                  VARIABLES
 ******************************************************************************/
static volatile bool device_need_delete = false;   // flag to delete device from cloud.

static mico_queue_t msg_recv_queue = NULL;   // fogcloud msg recv queue
static volatile uint32_t total_recv_buf_len = 0;
static mico_mutex_t msg_recv_queue_mutex = NULL;


/*******************************************************************************
 *                                  FUNCTIONS
 ******************************************************************************/
extern OSStatus MicoStartFogCloudConfigServer ( mico_Context_t * const inContext );
extern void set_RF_LED_cloud_connected     ( mico_Context_t * const inContext );
extern void set_RF_LED_cloud_disconnected  ( mico_Context_t * const inContext );

void MicoFogCloudNeedResetDevice(void)
{
  device_need_delete = true;
  return;
}

void fogNotify_WifiStatusHandler(WiFiEvent event, mico_Context_t * const inContext)
{
  fogcloud_log_trace();
  (void)inContext;
  switch (event) {
  case NOTIFY_STATION_UP:
    inContext->appStatus.isWifiConnected = true;
    break;
  case NOTIFY_STATION_DOWN:
    inContext->appStatus.isWifiConnected = false;
    break;
  case NOTIFY_AP_UP:
    break;
  case NOTIFY_AP_DOWN:
    break;
  default:
    break;
  }
  return;
}

#ifndef DISABLE_FOGCLOUD_OTA_CHECK
void fogcloud_ota_thread(void *arg)
{
  OSStatus err = kUnknownErr;
  MVDOTARequestData_t devOTARequestData;
  mico_Context_t *inContext = (mico_Context_t *)arg;
  
  fogcloud_log("OTA: check new firmware ...");
  memset((void*)&devOTARequestData, 0, sizeof(devOTARequestData));
  strncpy(devOTARequestData.loginId,
          inContext->flashContentInRam.appConfig.fogcloudConfig.loginId,
          MAX_SIZE_LOGIN_ID);
  strncpy(devOTARequestData.devPasswd,
          inContext->flashContentInRam.appConfig.fogcloudConfig.devPasswd,
          MAX_SIZE_DEV_PASSWD);
  strncpy(devOTARequestData.user_token,
          inContext->flashContentInRam.appConfig.fogcloudConfig.userToken,
          MAX_SIZE_USER_TOKEN);
  err = fogCloudDevFirmwareUpdate(inContext, devOTARequestData);
  if(kNoErr == err){
    if(inContext->appStatus.fogcloudStatus.RecvRomFileSize > 0){
      fogcloud_log("OTA: firmware download success, system will reboot && update...");
      // set bootloader to reboot && update app firmware
      mico_rtos_lock_mutex(&inContext->flashContentInRam_mutex);
      memset(&inContext->flashContentInRam.bootTable, 0, sizeof(boot_table_t));
      inContext->flashContentInRam.bootTable.length = inContext->appStatus.fogcloudStatus.RecvRomFileSize;
      inContext->flashContentInRam.bootTable.start_address = UPDATE_START_ADDRESS;
      inContext->flashContentInRam.bootTable.type = 'A';
      inContext->flashContentInRam.bootTable.upgrade_type = 'U';
      if(inContext->flashContentInRam.micoSystemConfig.configured != allConfigured)
        inContext->flashContentInRam.micoSystemConfig.easyLinkByPass = EASYLINK_SOFT_AP_BYPASS;
      MICOUpdateConfiguration(inContext);
      mico_rtos_unlock_mutex(&inContext->flashContentInRam_mutex);
      inContext->micoStatus.sys_state = eState_Software_Reset;
      if(inContext->micoStatus.sys_state_change_sem != NULL ){
        mico_rtos_set_semaphore(&inContext->micoStatus.sys_state_change_sem);
      }
      mico_thread_sleep(MICO_WAIT_FOREVER);
    }
    else{
      fogcloud_log("OTA: firmware is up-to-date!");
    }
  }
  else{
    fogcloud_log("OTA: firmware download failed, err=%d", err);
  }
  
  fogcloud_log("fogcloud_ota_thread exit err=%d.", err);
  mico_rtos_delete_thread(NULL);
  return;
}
#endif   // DISABLE_FOGCLOUD_OTA_CHECK

void fogcloud_main_thread(void *arg)
{
  OSStatus err = kUnknownErr;
  mico_Context_t *inContext = (mico_Context_t *)arg;
  
  MVDResetRequestData_t devResetRequestData;
  
#ifdef ENABLE_FOGCLOUD_AUTO_ACTIVATE
  MVDActivateRequestData_t devDefaultActivateData;
#endif
  
  /* wait for station on */
  while(!inContext->appStatus.isWifiConnected){
    mico_thread_msleep(500);
  }
  
  //--- create msg recv queue, NOTE: just push msg pionter into queue, so msg memory must be freed after used.
  if(NULL == msg_recv_queue_mutex){
    err = mico_rtos_init_mutex(&msg_recv_queue_mutex);
    require_noerr_action(err, exit,
                         fogcloud_log("ERROR: mico_rtos_init_mutex (msg_recv_queue_mutex) failed, err=%d.", err));
  }
  err = mico_rtos_init_queue(&msg_recv_queue, "fog_recv_queue", sizeof(int), FOGCLOUD_MAX_RECV_QUEUE_LENGTH);
  require_noerr_action(err, exit,
                       fogcloud_log("ERROR: mico_rtos_init_queue (msg_recv_queue) failed, err=%d", err));
  
  /* start FogCloud service */
  err = fogCloudStart(inContext);
  require_noerr_action(err, exit, 
                       fogcloud_log("ERROR: MicoFogCloudCloudInterfaceStart failed!") );
  
  /* start configServer for fogcloud (server for activate/authorize/reset/ota cmd from user APP) */
  if(false == inContext->flashContentInRam.appConfig.fogcloudConfig.owner_binding){
    err = MicoStartFogCloudConfigServer( inContext);
    require_noerr_action(err, exit, 
                         fogcloud_log("ERROR: start FogCloud configServer failed!") );
  }
  
 #ifdef ENABLE_FOGCLOUD_AUTO_ACTIVATE
  /* activate when wifi on */
  while(false == inContext->flashContentInRam.appConfig.fogcloudConfig.isActivated){
    // auto activate, using default login_id/dev_pass/user_token
    fogcloud_log("device activate start...");
    memset((void*)&devDefaultActivateData, 0, sizeof(devDefaultActivateData));
    strncpy(devDefaultActivateData.loginId,
            inContext->flashContentInRam.appConfig.fogcloudConfig.loginId,
            MAX_SIZE_LOGIN_ID);
    strncpy(devDefaultActivateData.devPasswd,
            inContext->flashContentInRam.appConfig.fogcloudConfig.devPasswd,
            MAX_SIZE_DEV_PASSWD);
    strncpy(devDefaultActivateData.user_token,
            inContext->micoStatus.mac,   // use MAC as default user_token
            MAX_SIZE_USER_TOKEN);
    err = fogCloudDevActivate(inContext, devDefaultActivateData);
    if(kNoErr == err){
      fogcloud_log("device activate success!");
      break;
    }
    else{
      fogcloud_log("device auto activate failed, err = %d, will retry in 3s ...", err);
    }
    mico_thread_sleep(3);
  }

#endif  // ENABLE_FOGCLOUD_AUTO_ACTIVATE
  
#ifndef DISABLE_FOGCLOUD_OTA_CHECK
  /* OTA check just device activated */
  if( (!inContext->appStatus.noOTACheckOnSystemStart) && 
     (inContext->flashContentInRam.appConfig.fogcloudConfig.isActivated) ){
    // start ota thread
    err = mico_rtos_create_thread(NULL, MICO_APPLICATION_PRIORITY, "fogcloud_ota", 
                                  fogcloud_ota_thread, STACK_SIZE_FOGCLOUD_OTA_THREAD, 
                                  inContext);
    if(kNoErr != err){
      fogcloud_log("ERROR: start FogCloud OTA thread failed, err=%d.", err);
    }
  }
  inContext->appStatus.noOTACheckOnSystemStart = false;
#endif  // DISABLE_FOGCLOUD_OTA_CHECK
  
  while(1){
    // device info reset
    if(device_need_delete){
      fogcloud_log("delete device from cloud ...");
      memset((void*)&devResetRequestData, 0, sizeof(devResetRequestData));
      strncpy(devResetRequestData.loginId,
              inContext->flashContentInRam.appConfig.fogcloudConfig.loginId,
              MAX_SIZE_LOGIN_ID);
      strncpy(devResetRequestData.devPasswd,
              inContext->flashContentInRam.appConfig.fogcloudConfig.devPasswd,
              MAX_SIZE_DEV_PASSWD);
      strncpy(devResetRequestData.user_token,
              inContext->flashContentInRam.appConfig.fogcloudConfig.userToken,
                MAX_SIZE_USER_TOKEN);
      err = fogCloudResetCloudDevInfo(inContext, devResetRequestData);
      if(kNoErr == err){
        device_need_delete = false;
        fogcloud_log("delete device success, system need reboot...");
        mico_rtos_lock_mutex(&inContext->flashContentInRam_mutex);
        MicoFogCloudRestoreDefault(inContext);
        MICOUpdateConfiguration(inContext);
        mico_rtos_unlock_mutex(&inContext->flashContentInRam_mutex);
        // system restart
        inContext->micoStatus.sys_state = eState_Software_Reset;
        if(inContext->micoStatus.sys_state_change_sem){
          mico_rtos_set_semaphore(&inContext->micoStatus.sys_state_change_sem);
        }
      }
      else{
        fogcloud_log("delete device failed, err = %d.", err);
      }
    }
    
    mico_thread_sleep(1);
    if(inContext->appStatus.fogcloudStatus.isOTAInProgress){
      continue;  // ota is in progress, the oled && system led will be holding
    }
    
    if(inContext->appStatus.fogcloudStatus.isCloudConnected){
      set_RF_LED_cloud_connected(inContext);  // toggle LED
    }
    else{
      set_RF_LED_cloud_disconnected(inContext);  // stop LED blink
    }
  }
  
exit:
  fogcloud_log("fogcloud_main_thread exit err=%d.", err);
  if(NULL != msg_recv_queue_mutex){
    mico_rtos_deinit_mutex(&msg_recv_queue_mutex);
  }
  if(NULL != msg_recv_queue){
    mico_rtos_deinit_queue(&msg_recv_queue);
  }
  mico_rtos_delete_thread(NULL);
  return;
}


/*******************************************************************************
 *                        FogCloud  interfaces init
 ******************************************************************************/
// reset default value
void MicoFogCloudRestoreDefault(mico_Context_t* const context)
{
  // reset all MicoFogCloud config params
  memset((void*)&(context->flashContentInRam.appConfig.fogcloudConfig), 
         0, sizeof(fogcloud_config_t));
  
  context->flashContentInRam.appConfig.fogcloudConfig.isActivated = false;
  context->flashContentInRam.appConfig.fogcloudConfig.owner_binding = false;
  sprintf(context->flashContentInRam.appConfig.fogcloudConfig.deviceId, DEFAULT_DEVICE_ID);
  sprintf(context->flashContentInRam.appConfig.fogcloudConfig.masterDeviceKey, DEFAULT_DEVICE_KEY);
  sprintf(context->flashContentInRam.appConfig.fogcloudConfig.romVersion, DEFAULT_ROM_VERSION);
  
  sprintf(context->flashContentInRam.appConfig.fogcloudConfig.loginId, DEFAULT_LOGIN_ID);
  sprintf(context->flashContentInRam.appConfig.fogcloudConfig.devPasswd, DEFAULT_DEV_PASSWD);
  sprintf(context->flashContentInRam.appConfig.fogcloudConfig.userToken, context->micoStatus.mac);
}

OSStatus MicoStartFogCloudService(mico_Context_t* const inContext)
{
  OSStatus err = kUnknownErr;
  
  //init MicoFogCloud status
  inContext->appStatus.fogcloudStatus.isCloudConnected = false;
  inContext->appStatus.fogcloudStatus.RecvRomFileSize = 0;
  inContext->appStatus.fogcloudStatus.isActivated = inContext->flashContentInRam.appConfig.fogcloudConfig.isActivated;
  inContext->appStatus.fogcloudStatus.isOTAInProgress = false;
  
  //init fogcloud service interface
  err = fogCloudInit(inContext);
  require_noerr_action(err, exit, 
                       fogcloud_log("ERROR: FogCloud interface init failed!") );
  
  err = MICOAddNotification( mico_notify_WIFI_STATUS_CHANGED, (void *)fogNotify_WifiStatusHandler );
  require_noerr_action(err, exit, 
                       fogcloud_log("ERROR: MICOAddNotification (mico_notify_WIFI_STATUS_CHANGED) failed!") );
  
  // start MicoFogCloud main thread (dev reset && ota check, then start fogcloud service)
  err = mico_rtos_create_thread(NULL, MICO_APPLICATION_PRIORITY, "fog_main", 
                                fogcloud_main_thread, STACK_SIZE_FOGCLOUD_MAIN_THREAD, 
                                inContext );
  
exit:
  return err;
}


/*******************************************************************************
*                            MicoFogCloud get state
*******************************************************************************/
// cloud connect state
bool MicoFogCloudIsConnect(mico_Context_t* const context)
{
  if(NULL == context){
    return false;
  }
  return context->appStatus.fogcloudStatus.isCloudConnected;
}

// device activate state
bool MicoFogCloudIsActivated(mico_Context_t* const context)
{
  if(NULL == context){
    return false;
  }
  return context->flashContentInRam.appConfig.fogcloudConfig.isActivated;
}


/*******************************************************************************
*                           FogCloud control interfaces
******************************************************************************/
//activate
OSStatus MicoFogCloudActivate(mico_Context_t* const context, 
                              MVDActivateRequestData_t activateData)
{
  OSStatus err = kUnknownErr;

  err = fogCloudDevActivate(context, activateData);
  require_noerr_action(err, exit, 
                       fogcloud_log("ERROR: device activate failed! err=%d", err) );
  return kNoErr;
  
exit:
  return err;
}

//authorize
OSStatus MicoFogCloudAuthorize(mico_Context_t* const context,
                               MVDAuthorizeRequestData_t authorizeData)
{
  OSStatus err = kUnknownErr;
  mico_Context_t *inContext = context;
  
  if(context->flashContentInRam.appConfig.fogcloudConfig.isActivated){
    err = fogCloudDevAuthorize(inContext, authorizeData);
    require_noerr_action(err, exit, 
                         fogcloud_log("ERROR: device authorize failed! err=%d", err) );
  }
  else{
    fogcloud_log("ERROR: device not activate!");
    err = kStateErr;
  }
  
exit:
  return err;
}

//OTA
OSStatus MicoFogCloudFirmwareUpdate(mico_Context_t* const context,
                                    MVDOTARequestData_t OTAData)
{
  OSStatus err = kUnknownErr;
  mico_Context_t *inContext = context;
  
  err = fogCloudDevFirmwareUpdate(inContext, OTAData);
  require_noerr_action(err, exit, 
                       fogcloud_log("ERROR: Firmware Update error! err=%d", err) );
  return kNoErr;
  
exit:
  return err;
}

//reset device info on cloud
OSStatus MicoFogCloudResetCloudDevInfo(mico_Context_t* const context,
                                       MVDResetRequestData_t devResetData)
{
  OSStatus err = kUnknownErr;
  mico_Context_t *inContext = context;
  
  err = fogCloudResetCloudDevInfo(inContext, devResetData);
  require_noerr_action(err, exit, 
                       fogcloud_log("ERROR: reset device info on cloud error! err=%d", err) );
  return kNoErr;
  
exit:
  return err;
}

//get state of the device( e.g. isActivate/isConnected)
OSStatus MicoFogCloudGetState(mico_Context_t* const context,
                              MVDGetStateRequestData_t getStateRequestData,
                              void* outDevState)
{
  //OSStatus err = kUnknownErr;
  mico_Context_t *inContext = context;
  json_object* report = (json_object*)outDevState;
  
  if((NULL == context) || (NULL == outDevState)){
    return kParamErr;
  }
  
  json_object_object_add(report, "isActivated",
                         json_object_new_boolean(inContext->flashContentInRam.appConfig.fogcloudConfig.isActivated)); 
    json_object_object_add(report, "isBinding",
                         json_object_new_boolean(inContext->flashContentInRam.appConfig.fogcloudConfig.owner_binding)); 
  json_object_object_add(report, "isConnected",
                         json_object_new_boolean(inContext->appStatus.fogcloudStatus.isCloudConnected));
  json_object_object_add(report, "version",
                         json_object_new_string(inContext->flashContentInRam.appConfig.fogcloudConfig.romVersion));
  
  return kNoErr;
}


/*******************************************************************************
*                       MicoFogCloud message send interface
******************************************************************************/
// MCU => Cloud
// if topic is NULL, send to default topic: device_id/out,
// else send to sub-channel: device_id/out/<topic>
OSStatus MicoFogCloudMsgSend(mico_Context_t* const context, const char* topic, 
                                   unsigned char *inBuf, unsigned int inBufLen)
{
  fogcloud_log_trace();
  OSStatus err = kUnknownErr;
  
  err = fogCloudSendtoChannel(topic, inBuf, inBufLen);  // transfer raw data
  require_noerr_action( err, exit, fogcloud_log("ERROR: send to cloud error! err=%d", err) );
  return kNoErr;
  
exit:
  return err;
}


/*******************************************************************************
* MicoFogCloud message exchange: push message into recv queue
******************************************************************************/
// handle cloud msg here, for example: send to USART or echo to cloud
OSStatus MicoFogCloudCloudMsgProcess(mico_Context_t* context, 
                                     const char* topic, const unsigned int topicLen,
                                     unsigned char *inBuf, unsigned int inBufLen)
{
  fogcloud_log_trace();
  OSStatus err = kUnknownErr;
  uint32_t real_msg_len = 0;

  // push msg into queue
  fogcloud_msg_t *real_msg;
  fogcloud_msg_t *p_tem_msg = NULL;  // msg pointer
  
  real_msg_len = sizeof(fogcloud_msg_t) + topicLen + inBufLen;
  if(FOGCLOUD_TOTAL_BUF_LENGTH < (total_recv_buf_len + real_msg_len)){
    return kNoMemoryErr;
  }
  real_msg = (fogcloud_msg_t*)malloc(real_msg_len);
  if (real_msg == NULL){
    return kNoMemoryErr;
  }
  total_recv_buf_len += real_msg_len;
  
	memset(real_msg, '\0', sizeof(real_msg));
  real_msg->topic_len = topicLen;
  real_msg->data_len = inBufLen;
  memcpy(real_msg->data, topic, topicLen);
  memcpy(real_msg->data + topicLen, inBuf, inBufLen);
  //memcpy(real_msg->data + topicLen + inBufLen, '\0', 1);
  
  if(NULL != msg_recv_queue){
    mico_rtos_lock_mutex(&msg_recv_queue_mutex);
    if(mico_rtos_is_queue_full(&msg_recv_queue)){
      fogcloud_log("WARNGING: FogCloud msg overrun, abandon old messages!");
      err = mico_rtos_pop_from_queue(&msg_recv_queue, &p_tem_msg, 0);  // just pop old msg pointer from queue 
      if(kNoErr == err){
        if(NULL != p_tem_msg){  // delete msg, free msg memory
          free(p_tem_msg);
          p_tem_msg = NULL;
        }
      }
      else{
        mico_rtos_unlock_mutex(&msg_recv_queue_mutex);
        fogcloud_log("WARNGING: FogCloud msg overrun, abandon current message!");
        if(NULL != real_msg){  // queue full, new msg abandoned
          free(real_msg);
          real_msg = NULL;
        }
        return kOverrunErr;
      }
    }
    // insert a msg into recv queue
    if (kNoErr != mico_rtos_push_to_queue(&msg_recv_queue, &real_msg, 0)) {  // just push msg pointer in queue
      free(real_msg);
      real_msg = NULL;
      err = kWriteErr;
    }
    else{
      err = kNoErr;
    }
    mico_rtos_unlock_mutex(&msg_recv_queue_mutex);
  }
  else{
    if(NULL != real_msg){
      free(real_msg);
      real_msg = NULL;
    }
    return kNotInitializedErr;
  }
  
  return err;
}

// recv msg from queue
OSStatus MicoFogCloudMsgRecv(mico_Context_t* const context, fogcloud_msg_t **msg, uint32_t timeout_ms)
{
  fogcloud_log_trace();
  OSStatus err = kUnknownErr;
  
  if(NULL == msg){
    return kParamErr;
  }
     
  if(NULL != msg_recv_queue){
    mico_rtos_lock_mutex(&msg_recv_queue_mutex);
    if(mico_rtos_is_queue_empty(&msg_recv_queue)){
      mico_rtos_unlock_mutex(&msg_recv_queue_mutex);
      return kUnderrunErr;
    }
    err = mico_rtos_pop_from_queue(&msg_recv_queue, msg, timeout_ms);  // just pop msg pointer from queue
    mico_rtos_unlock_mutex(&msg_recv_queue_mutex);
    if(kNoErr == err){
      total_recv_buf_len -= (sizeof(fogcloud_msg_t) - 1 + (*msg)->topic_len + (*msg)->data_len);
    }
  }
  else{
    err = kNotInitializedErr;
  }
  
  return err;
}
