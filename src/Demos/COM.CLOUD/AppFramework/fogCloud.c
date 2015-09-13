/**
******************************************************************************
* @file    fogcloud.c 
* @author  Eshen Wang
* @version V1.0.0
* @date    17-Mar-2015
* @brief   This file contains the implementations of low level 
*          interfaces for fogcloud.
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

#include "fogcloud.h"   
#include "FogCloudService.h"


#define cloud_if_log(M, ...) custom_log("FOGCLOUD_IF", M, ##__VA_ARGS__)
#define cloud_if_log_trace() custom_log_trace("FOGCLOUD_IF")


static easycloud_service_context_t easyCloudContext;

extern void OTAWillStart( mico_Context_t * const inContext );
extern void OTASuccess( mico_Context_t * const inContext );
extern void OTAFailed( mico_Context_t * const inContext );

//extern void  set_RF_LED_cloud_connected     ( mico_Context_t * const inContext );
//extern void  set_RF_LED_cloud_disconnected  ( mico_Context_t * const inContext );

WEAK OSStatus MicoFogCloudCloudMsgProcess(mico_Context_t* context, 
                                     const char* topic, const unsigned int topicLen,
                                     unsigned char *inBuf, unsigned int inBufLen)
{
  cloud_if_log("WARNING: MicoFogCloudCloudMsgProcess not override, use default functon. do nothing.");
  return kNoErr;
}

/*******************************************************************************
 * cloud service callbacks
 ******************************************************************************/

//cloud message recived handler
void cloudMsgArrivedHandler(void* context, 
                            const char* topic, const unsigned int topicLen,
                            unsigned char *msg, unsigned int msgLen)
{
  mico_Context_t *inContext = (mico_Context_t*)context;
  
  //note: get data just for length=len is valid, because Msg is just a buf pionter.
  cloud_if_log("Cloud[%.*s] => KIT: [%d]=%.*s", topicLen, topic, msgLen, msgLen, msg);
  
  MicoFogCloudCloudMsgProcess(inContext, topic, topicLen, msg, msgLen);
}

//cloud service status changed handler
void cloudServiceStatusChangedHandler(void* context, easycloud_service_status_t serviceStateInfo)
{
  mico_Context_t *inContext = (mico_Context_t*)context;

  if (FOGCLOUD_CONNECTED == serviceStateInfo.state){
    cloud_if_log("cloud service connected!");
    inContext->appStatus.fogcloudStatus.isCloudConnected = true;
    //set_RF_LED_cloud_connected(inContext);
  }
  else{
    cloud_if_log("cloud service disconnected!");
    inContext->appStatus.fogcloudStatus.isCloudConnected = false;
    //set_RF_LED_cloud_disconnected(inContext);
  }
}

OSStatus fogCloudPrintVersion(void)
{
  int cloudServiceLibVersion = 0;
  cloud_if_log("fogCloudPrintVersion");
  
  cloudServiceLibVersion = FogCloudServiceVersion(&easyCloudContext);
  UNUSED_PARAMETER(cloudServiceLibVersion);
  cloud_if_log("FogCloud library version: v%d.%d.%d", 
               (cloudServiceLibVersion & 0x00FF0000) >> 16,
               (cloudServiceLibVersion & 0x0000FF00) >> 8,
               (cloudServiceLibVersion & 0x000000FF));
  
  return kNoErr;
}

OSStatus fogCloudInit(mico_Context_t* const inContext)
{
  OSStatus err = kUnknownErr;
  int cloudServiceLibVersion = 0;
  
  // set cloud service config
  strncpy(easyCloudContext.service_config_info.bssid, 
          inContext->micoStatus.mac, MAX_SIZE_BSSID);
  strncpy(easyCloudContext.service_config_info.productId, 
          (char*)PRODUCT_ID, strlen((char*)PRODUCT_ID));
  strncpy(easyCloudContext.service_config_info.productKey, 
          (char*)PRODUCT_KEY, strlen((char*)PRODUCT_KEY));
  easyCloudContext.service_config_info.msgRecvhandler = cloudMsgArrivedHandler;
  easyCloudContext.service_config_info.statusNotify = cloudServiceStatusChangedHandler;
  easyCloudContext.service_config_info.context = (void*)inContext;
  
  // set cloud status
  memset((void*)&(easyCloudContext.service_status), '\0', sizeof(easyCloudContext.service_status));
  easyCloudContext.service_status.isActivated = inContext->flashContentInRam.appConfig.fogcloudConfig.isActivated;
  strncpy(easyCloudContext.service_status.deviceId, 
          inContext->flashContentInRam.appConfig.fogcloudConfig.deviceId, MAX_SIZE_DEVICE_ID);
  strncpy(easyCloudContext.service_status.masterDeviceKey, 
          inContext->flashContentInRam.appConfig.fogcloudConfig.masterDeviceKey, MAX_SIZE_DEVICE_KEY);
  strncpy(easyCloudContext.service_status.device_name, 
          DEFAULT_DEVICE_NAME, MAX_SIZE_DEVICE_NAME);
  
  cloudServiceLibVersion = FogCloudServiceVersion(&easyCloudContext);
  UNUSED_PARAMETER(cloudServiceLibVersion);
  cloud_if_log("FogCloud library version: %d.%d.%d", 
               (cloudServiceLibVersion & 0x00FF0000) >> 16,
               (cloudServiceLibVersion & 0x0000FF00) >> 8,
               (cloudServiceLibVersion & 0x000000FF));
  
  err = FogCloudServiceInit(&easyCloudContext);
  require_noerr_action( err, exit, cloud_if_log("ERROR: FogCloud service init failed.") );
  return kNoErr;
  
exit:
  return err; 
}


OSStatus fogCloudStart(mico_Context_t* const inContext)
{
  OSStatus err = kUnknownErr;
  
  if(NULL == inContext){
    return kParamErr;
  }
  
  // start cloud service
  err = FogCloudServiceStart(&easyCloudContext);
  require_noerr_action( err, exit, cloud_if_log("ERROR: FogCloud service start failed.") );
  return kNoErr;
  
exit:
  return err;
}

easycloud_service_state_t fogCloudGetState(void)
{
  easycloud_service_state_t service_running_state = FOGCLOUD_STOPPED;
  
  cloud_if_log("fogCloudGetState");
  service_running_state = FogCloudServiceState(&easyCloudContext);
  return service_running_state;
}

OSStatus fogCloudSend(unsigned char *inBuf, unsigned int inBufLen)
{
  cloud_if_log_trace();
  OSStatus err = kUnknownErr;

  cloud_if_log("KIT => Cloud[publish]:[%d]=%.*s", inBufLen, inBufLen, inBuf);
  err = FogCloudPublish(&easyCloudContext, inBuf, inBufLen);
  require_noerr_action( err, exit, cloud_if_log("ERROR: fogCloudSend failed! err=%d", err) );
  return kNoErr;
  
exit:
  return err;
}

OSStatus fogCloudSendto(const char* topic, unsigned char *inBuf, unsigned int inBufLen)
{
  cloud_if_log_trace();
  OSStatus err = kUnknownErr;

  cloud_if_log("KIT => Cloud[%s]:[%d]=%.*s", topic, inBufLen, inBufLen, inBuf);
  err = FogCloudPublishto(&easyCloudContext, topic, inBuf, inBufLen);
  require_noerr_action( err, exit, cloud_if_log("ERROR: fogCloudSendto failed! err=%d", err) );
  return kNoErr;
  
exit:
  return err;
}

OSStatus fogCloudSendtoChannel(const char* channel, unsigned char *inBuf, unsigned int inBufLen)
{
  cloud_if_log_trace();
  OSStatus err = kUnknownErr;

  cloud_if_log("KIT => Cloud[%s]:[%d]=%.*s", channel, inBufLen, inBufLen, inBuf);
  err = FogCloudPublishtoChannel(&easyCloudContext, channel, inBuf, inBufLen);
  require_noerr_action( err, exit, cloud_if_log("ERROR: fogCloudSendtoChannel failed! err=%d", err) );
  return kNoErr;
  
exit:
  return err;
}

OSStatus fogCloudDevActivate(mico_Context_t* const inContext,
                                      MVDActivateRequestData_t devActivateRequestData)
{
  cloud_if_log_trace();
  OSStatus err = kUnknownErr;
  
  cloud_if_log("Device activate...");
  
  //ok, set cloud context
  strncpy(easyCloudContext.service_config_info.loginId, 
          devActivateRequestData.loginId, MAX_SIZE_LOGIN_ID);
  strncpy(easyCloudContext.service_config_info.devPasswd, 
          devActivateRequestData.devPasswd, MAX_SIZE_DEV_PASSWD);
  strncpy(easyCloudContext.service_config_info.userToken, 
          devActivateRequestData.user_token, MAX_SIZE_USER_TOKEN);
    
  // activate request
  err = FogCloudActivate(&easyCloudContext);
  require_noerr_action(err, exit, 
                       cloud_if_log("ERROR: fogCloudDevActivate failed! err=%d", err) );
  
  inContext->appStatus.fogcloudStatus.isActivated = true;
  
  // write activate data back to flash
  mico_rtos_lock_mutex(&inContext->flashContentInRam_mutex);
  inContext->flashContentInRam.appConfig.fogcloudConfig.isActivated = true;
  strncpy(inContext->flashContentInRam.appConfig.fogcloudConfig.deviceId,
          easyCloudContext.service_status.deviceId, MAX_SIZE_DEVICE_ID);
  strncpy(inContext->flashContentInRam.appConfig.fogcloudConfig.masterDeviceKey,
          easyCloudContext.service_status.masterDeviceKey, MAX_SIZE_DEVICE_KEY);
  
  strncpy(inContext->flashContentInRam.appConfig.fogcloudConfig.loginId,
          easyCloudContext.service_config_info.loginId, MAX_SIZE_LOGIN_ID);
  strncpy(inContext->flashContentInRam.appConfig.fogcloudConfig.devPasswd,
          easyCloudContext.service_config_info.devPasswd, MAX_SIZE_DEV_PASSWD);
  strncpy(inContext->flashContentInRam.appConfig.fogcloudConfig.userToken,
          easyCloudContext.service_config_info.userToken, MAX_SIZE_USER_TOKEN);
    
  err = MICOUpdateConfiguration(inContext);
  mico_rtos_unlock_mutex(&inContext->flashContentInRam_mutex);
  require_noerr_action(err, exit, 
                       cloud_if_log("ERROR: activate write flash failed! err=%d", err) );
  
  return kNoErr;
  
exit:
  return err;
}

OSStatus fogCloudDevAuthorize(mico_Context_t* const inContext,
                                       MVDAuthorizeRequestData_t devAuthorizeReqData)
{
  cloud_if_log_trace();
  OSStatus err = kUnknownErr;
  easycloud_service_state_t cloudServiceState = FOGCLOUD_STOPPED;
  
  cloud_if_log("Device authorize...");

  cloudServiceState = FogCloudServiceState(&easyCloudContext);
  if (FOGCLOUD_STOPPED == cloudServiceState){
    return kStateErr;
  }
  
  //ok, set cloud context
  strncpy(easyCloudContext.service_config_info.loginId, 
          devAuthorizeReqData.loginId, MAX_SIZE_LOGIN_ID);
  strncpy(easyCloudContext.service_config_info.devPasswd, 
          devAuthorizeReqData.devPasswd, MAX_SIZE_DEV_PASSWD);
  strncpy(easyCloudContext.service_config_info.userToken, 
          devAuthorizeReqData.user_token, MAX_SIZE_USER_TOKEN);
  
  err = FogCloudAuthorize(&easyCloudContext);
  require_noerr_action( err, exit, cloud_if_log("ERROR: authorize failed! err=%d", err) );
  return kNoErr;
  
exit:
  return err;
}


#define SizePerRW 1024   /* Bootloader need 2xSizePerRW RAM heap size to operate, 
                            but it can boost the setup. */

OSStatus fogCloudDevFirmwareUpdate(mico_Context_t* const inContext,
                                            MVDOTARequestData_t devOTARequestData)
{
  cloud_if_log_trace();
  OSStatus err = kUnknownErr;
  ecs_ota_flash_params_t ota_flash_params = {
    MICO_FLASH_FOR_UPDATE,
    UPDATE_START_ADDRESS,
    UPDATE_END_ADDRESS,
    UPDATE_FLASH_SIZE
  };
  
  md5_context md5;
  unsigned char md5_16[16] = {0};
  char *pmd5_32 = NULL;
  char rom_file_md5[32] = {0};
  uint8_t data[SizePerRW] = {0};
  uint32_t updateStartAddress = 0;
  uint32_t readLength = 0;
  uint32_t i = 0, size = 0;
  
  bool flash_initialized = false;
  
  //get latest rom version, file_path, md5
  cloud_if_log("fogCloudDevFirmwareUpdate: get latest rom version from server ...");
  err = FogCloudGetLatestRomVersion(&easyCloudContext);
  require_noerr_action( err, exit_with_error, cloud_if_log("ERROR: FogCloudGetLatestRomVersion failed! err=%d", err) );
  
  //FW version compare
  cloud_if_log("currnt_version=%s", inContext->flashContentInRam.appConfig.fogcloudConfig.romVersion);
  cloud_if_log("latestRomVersion=%s", easyCloudContext.service_status.latestRomVersion);
  cloud_if_log("bin_file=%s", easyCloudContext.service_status.bin_file);
  cloud_if_log("bin_md5=%s", easyCloudContext.service_status.bin_md5);
  
  if(0 == strncmp(inContext->flashContentInRam.appConfig.fogcloudConfig.romVersion,
                  easyCloudContext.service_status.latestRomVersion, 
                  strlen(inContext->flashContentInRam.appConfig.fogcloudConfig.romVersion))){
     cloud_if_log("the current firmware version[%s] is up-to-date!", 
                  inContext->flashContentInRam.appConfig.fogcloudConfig.romVersion);
     inContext->appStatus.fogcloudStatus.RecvRomFileSize = 0;
     err = kNoErr;
     goto exit_with_no_error;
  }
  cloud_if_log("fogCloudDevFirmwareUpdate: new firmware[%s] found on server, downloading ...",
               easyCloudContext.service_status.latestRomVersion);
  
  inContext->appStatus.fogcloudStatus.isOTAInProgress = true;
  OTAWillStart(inContext);
  
  //get rom data
  err = FogCloudGetRomData(&easyCloudContext, ota_flash_params);
  require_noerr_action( err, exit_with_error, 
                       cloud_if_log("ERROR: FogCloudGetRomData failed! err=%d", err) );
  
  //------------------------------ OTA DATA VERIFY -----------------------------
  // md5 init
  InitMd5(&md5);
  memset(rom_file_md5, 0, 32);
  memset(data, 0xFF, SizePerRW);
  err = MicoFlashInitialize( MICO_FLASH_FOR_UPDATE );
  require_noerr(err, exit_with_error);
  flash_initialized = true;
  updateStartAddress = UPDATE_START_ADDRESS;
  size = (easyCloudContext.service_status.bin_file_size)/SizePerRW;
  
  // read flash, md5 update
  for(i = 0; i <= size; i++){
    if( i == size ){
      if( (easyCloudContext.service_status.bin_file_size)%SizePerRW ){
        readLength = (easyCloudContext.service_status.bin_file_size)%SizePerRW;
      }
      else{
        break;
      }
    }
    else{
      readLength = SizePerRW;
    }
    err = MicoFlashRead(MICO_FLASH_FOR_UPDATE, &updateStartAddress, data, readLength);
    require_noerr(err, exit_with_error);
    Md5Update(&md5, (uint8_t *)data, readLength);
  } 
  if(flash_initialized){
    MicoFlashFinalize(MICO_FLASH_FOR_UPDATE);
    flash_initialized = false;
  }
  
 // read done, calc MD5
  Md5Final(&md5, md5_16);
  pmd5_32 = ECS_DataToHexStringLowercase(md5_16,  sizeof(md5_16));  //convert hex data to hex string
  cloud_if_log("ota_data_in_flash_md5[%d]=%s", strlen(pmd5_32), pmd5_32);
  
  if (NULL != pmd5_32){
    strncpy(rom_file_md5, pmd5_32, strlen(pmd5_32));
    free(pmd5_32);
    pmd5_32 = NULL;
  }
  else{
    err = kNoMemoryErr;
    goto exit_with_error;
  }
  
  // check md5
  if(0 != strncmp( easyCloudContext.service_status.bin_md5, (char*)&(rom_file_md5[0]), 
                  strlen( easyCloudContext.service_status.bin_md5))){
    cloud_if_log("ERROR: ota data wrote in flash md5 checksum err!!!");
    err = kChecksumErr;
    goto exit_with_error;
   }
  //----------------------------------------------------------------------------
  
  //update rom version in flash
  cloud_if_log("fogCloudDevFirmwareUpdate: return rom version && file size.");
  mico_rtos_lock_mutex(&inContext->flashContentInRam_mutex);
  memset(inContext->flashContentInRam.appConfig.fogcloudConfig.romVersion,
         0, MAX_SIZE_FW_VERSION);
  strncpy(inContext->flashContentInRam.appConfig.fogcloudConfig.romVersion,
          easyCloudContext.service_status.latestRomVersion, 
          strlen(easyCloudContext.service_status.latestRomVersion));
  inContext->appStatus.fogcloudStatus.RecvRomFileSize = easyCloudContext.service_status.bin_file_size;
  MICOUpdateConfiguration(inContext);
  mico_rtos_unlock_mutex(&inContext->flashContentInRam_mutex);
  
  OTASuccess(inContext);
  err = kNoErr;
  goto exit_with_no_error;
  
exit_with_no_error:
  cloud_if_log("fogCloudDevFirmwareUpdate exit with no error.");
  if(flash_initialized){
    MicoFlashFinalize(MICO_FLASH_FOR_UPDATE);
    flash_initialized = false;
  }
  inContext->appStatus.fogcloudStatus.isOTAInProgress = false;
  return err;
  
exit_with_error:
  cloud_if_log("fogCloudDevFirmwareUpdate exit with err=%d.", err);
  if(flash_initialized){
    MicoFlashFinalize(MICO_FLASH_FOR_UPDATE);
    flash_initialized = false;
  }
  OTAFailed(inContext);
  inContext->appStatus.fogcloudStatus.isOTAInProgress = false;
  return err;
}

OSStatus fogCloudResetCloudDevInfo(mico_Context_t* const inContext,
                                            MVDResetRequestData_t devResetRequestData)
{
  OSStatus err = kUnknownErr;
  
  cloud_if_log("Delete device info from cloud...");
    
  err = FogCloudDeviceReset(&easyCloudContext);
  require_noerr_action( err, exit, cloud_if_log("ERROR: FogCloudDeviceReset failed! err=%d", err) );
  
  inContext->appStatus.fogcloudStatus.isActivated = false;
  
  mico_rtos_lock_mutex(&inContext->flashContentInRam_mutex);
  inContext->flashContentInRam.appConfig.fogcloudConfig.isActivated = false;  // need to reActivate
  inContext->flashContentInRam.appConfig.fogcloudConfig.owner_binding = false;  // no owner binding
  sprintf(inContext->flashContentInRam.appConfig.fogcloudConfig.deviceId, DEFAULT_DEVICE_ID);
  sprintf(inContext->flashContentInRam.appConfig.fogcloudConfig.masterDeviceKey, DEFAULT_DEVICE_KEY);
  sprintf(inContext->flashContentInRam.appConfig.fogcloudConfig.loginId, DEFAULT_LOGIN_ID);
  sprintf(inContext->flashContentInRam.appConfig.fogcloudConfig.devPasswd, DEFAULT_DEV_PASSWD);
  inContext->appStatus.fogcloudStatus.isCloudConnected = false;
  MICOUpdateConfiguration(inContext);
  mico_rtos_unlock_mutex(&inContext->flashContentInRam_mutex);
  
exit:
  return err;
}

OSStatus fogCloudStop(mico_Context_t* const inContext)
{  
  cloud_if_log_trace();
  OSStatus err = kUnknownErr;
  
  cloud_if_log("fogCloudStop");
  err = FogCloudServiceStop(&easyCloudContext);
  require_noerr_action( err, exit, 
                       cloud_if_log("ERROR: FogCloudServiceStop err=%d.", err) );
  return kNoErr;
  
exit:
  return err;
}

OSStatus fogCloudDeinit(mico_Context_t* const inContext)
{  
  cloud_if_log_trace();
  OSStatus err = kUnknownErr;
  
  cloud_if_log("fogCloudDeinit");
  err = FogCloudServiceDeInit(&easyCloudContext);
  require_noerr_action( err, exit, 
                       cloud_if_log("ERROR: FogCloudServiceDeInit err=%d.", err) );
  return kNoErr;
  
exit:
  return err;
}
