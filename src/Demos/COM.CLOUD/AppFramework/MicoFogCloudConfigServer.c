/**
******************************************************************************
* @file MicoFogCloudConfigServer.c
* @author Eshen Wang
* @version V1.0.0
* @date 17-Mar-2015
* @brief Local TCP server for fogcloud configuration
******************************************************************************
*
* The MIT License
* Copyright (c) 2014 MXCHIP Inc.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is furnished
* to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
* IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
******************************************************************************
*/

#include "MICO.h"
#include "MICODefine.h"
#include "SocketUtils.h"
#include "MDNSUtils.h"
#include "StringUtils.h"
#include "MicoFogCloud.h"

#define fogcloud_config_log(M, ...) custom_log("FogCloud_ConfigServer", M, ##__VA_ARGS__)
#define fogcloud_config_log_trace() custom_log_trace("FogCloud_ConfigServer")

#define STACK_SIZE_FOGCLOUD_CONFIG_SERVER_THREAD   0x300
#define STACK_SIZE_FOGCLOUD_CONFIG_CLIENT_THREAD   0xD00

#define kCONFIGURLDevState               "/dev-state"
#define kCONFIGURLDevActivate            "/dev-activate"
#define kCONFIGURLDevAuthorize           "/dev-authorize"
#define kCONFIGURLResetCloudDevInfo      "/dev-cloud_reset"
#define kCONFIGURLDevFWUpdate            "/dev-fw_update"

extern json_object* ConfigCreateReportJsonMessage( mico_Context_t * const inContext );
extern OSStatus getMVDActivateRequestData(const char *input, MVDActivateRequestData_t *activateData);
extern OSStatus getMVDAuthorizeRequestData(const char *input, MVDAuthorizeRequestData_t *authorizeData);
extern OSStatus getMVDResetRequestData(const char *input, MVDResetRequestData_t *devResetData);
extern OSStatus getMVDOTARequestData(const char *input, MVDOTARequestData_t *OTAData);
extern OSStatus getMVDGetStateRequestData(const char *input, MVDGetStateRequestData_t *devGetStateData);

static void fogCloudConfigServer_listener_thread(void *inContext);
static void fogCloudConfigClient_thread(void *inFd);
static mico_Context_t *Context;
static volatile bool fog_config_server_running = true;
static OSStatus _LocalConfigRespondInComingMessage(int fd, ECS_HTTPHeader_t* inHeader, 
                                                   mico_Context_t * const inContext);

OSStatus MicoStartFogCloudConfigServer ( mico_Context_t * const inContext )
{
  return mico_rtos_create_thread(NULL, MICO_APPLICATION_PRIORITY, 
                                 "fog_server", 
                                 fogCloudConfigServer_listener_thread, 
                                 STACK_SIZE_FOGCLOUD_CONFIG_SERVER_THREAD, (void*)inContext );
}

void fogCloudConfigServer_listener_thread(void *inContext)
{
  fogcloud_config_log_trace();
  OSStatus err = kUnknownErr;
  int j;
  Context = inContext;
  struct sockaddr_t addr;
  int sockaddr_t_size;
  fd_set readfds;
  char ip_address[16];
  int localConfiglistener_fd = -1;
  
  /*Establish a TCP server fd that accept the tcp clients connections*/
  localConfiglistener_fd = socket( AF_INET, SOCK_STREAM, IPPROTO_TCP );
  require_action(IsValidSocket( localConfiglistener_fd ), exit, err = kNoResourcesErr );
  addr.s_ip = INADDR_ANY;
  addr.s_port = FOGCLOUD_CONFIG_SERVER_PORT;
  err = bind(localConfiglistener_fd, &addr, sizeof(addr));
  require_noerr( err, exit );
  err = listen(localConfiglistener_fd, 0);
  require_noerr( err, exit );
  fogcloud_config_log("fogCloud Config Server established at port: %d, fd: %d", 
                      FOGCLOUD_CONFIG_SERVER_PORT, localConfiglistener_fd);
  
  while(1){
    if(false == fog_config_server_running){
      break;
    }
    
    FD_ZERO(&readfds);
    FD_SET(localConfiglistener_fd, &readfds);
    select(1, &readfds, NULL, NULL, NULL);
    
    /*Check tcp connection requests */
    if(FD_ISSET(localConfiglistener_fd, &readfds)){
      sockaddr_t_size = sizeof(struct sockaddr_t);
      j = accept(localConfiglistener_fd, &addr, &sockaddr_t_size);
      if (j > 0) {
        inet_ntoa(ip_address, addr.s_ip );
        fogcloud_config_log("fogCloud Config Client %s:%d connected, fd: %d", ip_address, addr.s_port, j);
        err = mico_rtos_create_thread(NULL, MICO_APPLICATION_PRIORITY, "fog_client", 
                                      fogCloudConfigClient_thread, STACK_SIZE_FOGCLOUD_CONFIG_CLIENT_THREAD, &j);
      }
    }
  }
  
exit:
  fogcloud_config_log("Exit: Fog config Server exit with err = %d", err);
  SocketClose(&localConfiglistener_fd);
  mico_rtos_delete_thread(NULL);
  return;
}

void fogCloudConfigClient_thread(void *inFd)
{
  OSStatus err;
  int clientFd = *(int *)inFd;
  int clientFdIsSet;
  fd_set readfds;
  struct timeval_t t;
  ECS_HTTPHeader_t *httpHeader = NULL;
  fogcloud_config_log_trace();
  httpHeader = ECS_HTTPHeaderCreate();
  require_action( httpHeader, exit, err = kNoMemoryErr );
  ECS_HTTPHeaderClear( httpHeader );
  t.tv_sec = 10;
  t.tv_usec = 0;
  while(1){
    FD_ZERO(&readfds);
    FD_SET(clientFd, &readfds);
    clientFdIsSet = 0;
    if(httpHeader->len == 0){
      err = select(1, &readfds, NULL, NULL, &t);
      require( err > 0, exit );
      clientFdIsSet = FD_ISSET(clientFd, &readfds);
    }
    if(clientFdIsSet||httpHeader->len){
      err = ECS_SocketReadHTTPHeader( clientFd, httpHeader );
      fogcloud_config_log("httpHeader: %s", httpHeader->buf);
      switch ( err )
      {
      case kNoErr:
        // Read the rest of the HTTP body if necessary
        err = ECS_SocketReadHTTPBody( clientFd, httpHeader );
        if(httpHeader->dataEndedbyClose == true){
          err = _LocalConfigRespondInComingMessage( clientFd, httpHeader, Context );
          require_noerr(err, exit);
          err = kConnectionErr;
          goto exit;
        }else{
          require_noerr(err, exit);
          err = _LocalConfigRespondInComingMessage( clientFd, httpHeader, Context );
          //require_noerr(err, exit);
          goto exit;
        }
      case EWOULDBLOCK:
        // NO-OP, keep reading
        break;
      case kNoSpaceErr:
        fogcloud_config_log("ERROR: Cannot fit HTTPHeader.");
        goto exit;
      case kConnectionErr:
        // NOTE: kConnectionErr from ECS_SocketReadHTTPHeader means it's closed
        fogcloud_config_log("ERROR: Connection closed.");
        goto exit;
      default:
        fogcloud_config_log("ERROR: HTTP Header parse internal error: %d", err);
        goto exit;
      }
    }
  }
  
exit:
  if(kNoErr != err){
    fogcloud_config_log("Exit: fogCloud config client exit with err = %d", err);
  }
  else{
    fogcloud_config_log("Exit: fogcloud config client exit with success!");
  }
  SocketClose(&clientFd);
  ECS_HTTPHeaderClear( httpHeader );
  if(httpHeader) free(httpHeader);
  mico_rtos_delete_thread(NULL);
  return;
}

OSStatus _LocalConfigRespondInComingMessage(int fd, ECS_HTTPHeader_t* inHeader, mico_Context_t * const inContext)
{
  OSStatus err = kUnknownErr;
  const char * json_str;
  uint8_t *httpResponse = NULL;
  size_t httpResponseLen = 0;
  json_object* report = NULL;
  char err_msg[32] = {0};
  char *bonjour_txt_record = NULL;
  char *bonjour_txt_field = NULL;
  
  MVDActivateRequestData_t devActivateRequestData;
  MVDAuthorizeRequestData_t devAuthorizeRequestData;
  MVDResetRequestData_t devResetRequestData;
  MVDOTARequestData_t devOTARequestData;
  MVDGetStateRequestData_t devGetStateRequestData;
  
  fogcloud_config_log_trace();
  
  if(ECS_HTTPHeaderMatchURL( inHeader, kCONFIGURLDevState ) == kNoErr){
    if(inHeader->contentLength > 0){
      fogcloud_config_log("Recv device getState request.");
      memset((void*)&devGetStateRequestData, '\0', sizeof(devGetStateRequestData));
      err = getMVDGetStateRequestData(inHeader->extraDataPtr, &devGetStateRequestData);
      require_noerr( err, exit );
      report = json_object_new_object();
      err = MicoFogCloudGetState(inContext, devGetStateRequestData, report);
      require_noerr( err, exit );
      fogcloud_config_log("get device state success!");
      json_str = (char*)json_object_to_json_string(report);
      //config_log("json_str=%s", json_str);
      err = ECS_CreateSimpleHTTPMessage( ECS_kMIMEType_JSON, (uint8_t*)json_str, strlen(json_str),
                                        &httpResponse, &httpResponseLen );
      require( httpResponse, exit );
      err = SocketSend( fd, httpResponse, httpResponseLen );
      SocketClose(&fd);
    }
    goto exit;
  }
  else if(ECS_HTTPHeaderMatchURL( inHeader, kCONFIGURLDevActivate ) == kNoErr){
    if(inHeader->contentLength > 0){
      fogcloud_config_log("Recv device activate request.");
      memset((void*)&devActivateRequestData, '\0', sizeof(devActivateRequestData));
      err = getMVDActivateRequestData(inHeader->extraDataPtr, &devActivateRequestData);
      require_noerr( err, exit );
      err = MicoFogCloudActivate(inContext, devActivateRequestData);
      require_noerr( err, exit );
      fogcloud_config_log("Device activate success!");
      //------------------------------------------------------------------------
      fog_config_server_running = false;  // stop fog config server
      fogcloud_config_log("update bonjour txt record.");
      // update owner binding flag in txt record of bonjour
      suspend_bonjour_service(true);
      mico_rtos_lock_mutex(&inContext->flashContentInRam_mutex);
      inContext->flashContentInRam.appConfig.fogcloudConfig.owner_binding = true;
      err = MICOUpdateConfiguration(inContext);
      mico_rtos_unlock_mutex(&inContext->flashContentInRam_mutex);
      
      bonjour_txt_record = malloc(550);
      require_action(bonjour_txt_record, exit, err = kNoMemoryErr);
      
      bonjour_txt_field = __strdup_trans_dot(inContext->micoStatus.mac);
      sprintf(bonjour_txt_record, "MAC=%s.", bonjour_txt_field);
      free(bonjour_txt_field);
      
      bonjour_txt_field = __strdup_trans_dot((inContext->flashContentInRam.appConfig.fogcloudConfig.owner_binding) ? "true" : "false");
      sprintf(bonjour_txt_record, "%sBinding=%s.", bonjour_txt_record, bonjour_txt_field);
      free(bonjour_txt_field);
      
      bonjour_txt_field = __strdup_trans_dot(FIRMWARE_REVISION);
      sprintf(bonjour_txt_record, "%sFirmware Rev=%s.", bonjour_txt_record, bonjour_txt_field);
      free(bonjour_txt_field);
      
      bonjour_txt_field = __strdup_trans_dot(HARDWARE_REVISION);
      sprintf(bonjour_txt_record, "%sHardware Rev=%s.", bonjour_txt_record, bonjour_txt_field);
      free(bonjour_txt_field);
      
      bonjour_txt_field = __strdup_trans_dot(MicoGetVer());
      sprintf(bonjour_txt_record, "%sMICO OS Rev=%s.", bonjour_txt_record, bonjour_txt_field);
      free(bonjour_txt_field);
      
      bonjour_txt_field = __strdup_trans_dot(MODEL);
      sprintf(bonjour_txt_record, "%sModel=%s.", bonjour_txt_record, bonjour_txt_field);
      free(bonjour_txt_field);
      
      bonjour_txt_field = __strdup_trans_dot(PROTOCOL);
      sprintf(bonjour_txt_record, "%sProtocol=%s.", bonjour_txt_record, bonjour_txt_field);
      free(bonjour_txt_field);
      
      bonjour_txt_field = __strdup_trans_dot(MANUFACTURER);
      sprintf(bonjour_txt_record, "%sManufacturer=%s.", bonjour_txt_record, bonjour_txt_field);
      free(bonjour_txt_field);
      
      sprintf(bonjour_txt_record, "%sSeed=%u.", bonjour_txt_record, inContext->flashContentInRam.micoSystemConfig.seed);
      
      bonjour_update_txt_record(bonjour_txt_record);
      if(NULL != bonjour_txt_record) free(bonjour_txt_record);
      suspend_bonjour_service(false);
      //------------------------------------------------------------------------
      report = json_object_new_object();
      require_action(report, exit, err = kNoMemoryErr);
      json_object_object_add(report, "device_id",
                             json_object_new_string(inContext->flashContentInRam.appConfig.fogcloudConfig.deviceId));
      json_str = (char*)json_object_to_json_string(report);
      //config_log("json_str=%s", json_str);
      err = ECS_CreateSimpleHTTPMessage( ECS_kMIMEType_JSON, (uint8_t*)json_str, strlen(json_str),
                                        &httpResponse, &httpResponseLen );
      require_noerr( err, exit );
      require( httpResponse, exit );
      err = SocketSend( fd, httpResponse, httpResponseLen );
      SocketClose(&fd);
    }
    goto exit;
  }
  else if(ECS_HTTPHeaderMatchURL( inHeader, kCONFIGURLDevAuthorize ) == kNoErr){
    if(inHeader->contentLength > 0){
      fogcloud_config_log("Recv device authorize request.");
      memset((void*)&devAuthorizeRequestData, '\0', sizeof(devAuthorizeRequestData));
      err = getMVDAuthorizeRequestData( inHeader->extraDataPtr, &devAuthorizeRequestData);
      require_noerr( err, exit );
      err = MicoFogCloudAuthorize(inContext, devAuthorizeRequestData);
      require_noerr( err, exit );
      fogcloud_config_log("Device authorize success!");
      report = json_object_new_object();
      require_action(report, exit, err = kNoMemoryErr);
      json_object_object_add(report, "device_id",
                             json_object_new_string(inContext->flashContentInRam.appConfig.fogcloudConfig.deviceId));
      json_str = (char*)json_object_to_json_string(report);
      //config_log("json_str=%s", json_str);
      err = ECS_CreateSimpleHTTPMessage( ECS_kMIMEType_JSON, (uint8_t*)json_str, strlen(json_str),
                                        &httpResponse, &httpResponseLen );
      require( httpResponse, exit );
      err = SocketSend( fd, httpResponse, httpResponseLen );
      SocketClose(&fd);
    }
    goto exit;
  }
  else if(ECS_HTTPHeaderMatchURL( inHeader, kCONFIGURLResetCloudDevInfo ) == kNoErr){
    if(inHeader->contentLength > 0){
      fogcloud_config_log("Recv cloud device info reset request.");
      memset((void*)&devResetRequestData, '\0', sizeof(devResetRequestData));
      err = getMVDResetRequestData( inHeader->extraDataPtr, &devResetRequestData);
      require_noerr( err, exit );
      err = MicoFogCloudResetCloudDevInfo(inContext, devResetRequestData);
      require_noerr( err, exit );
      fogcloud_config_log("Device cloud reset success!");
      err = ECS_CreateSimpleHTTPOKMessage( &httpResponse, &httpResponseLen );
      require_noerr( err, exit );
      require( httpResponse, exit );
      err = SocketSend( fd, httpResponse, httpResponseLen );
      SocketClose(&fd);
     
      inContext->micoStatus.sys_state = eState_Software_Reset;
      require(inContext->micoStatus.sys_state_change_sem, exit);
      mico_rtos_set_semaphore(&inContext->micoStatus.sys_state_change_sem);
    }
    goto exit;
  }
  else if(ECS_HTTPHeaderMatchURL( inHeader, kCONFIGURLDevFWUpdate ) == kNoErr){
    if(inHeader->contentLength > 0){
      fogcloud_config_log("Recv device fw_update request.");
      memset((void*)&devOTARequestData, '\0', sizeof(devOTARequestData));
      err = getMVDOTARequestData( inHeader->extraDataPtr, &devOTARequestData);
      require_noerr( err, exit );
      err = MicoFogCloudFirmwareUpdate(inContext, devOTARequestData);
      require_noerr( err, exit );
      fogcloud_config_log("Device firmware update success!");
      err = ECS_CreateSimpleHTTPOKMessage( &httpResponse, &httpResponseLen );
      require_noerr( err, exit );
      require( httpResponse, exit );
      err = SocketSend( fd, httpResponse, httpResponseLen );
      SocketClose(&fd);
      fogcloud_config_log("OTA bin_size=%lld, bin_version=%s",
                          inContext->appStatus.fogcloudStatus.RecvRomFileSize,
                          inContext->flashContentInRam.appConfig.fogcloudConfig.romVersion );
      if(0 == inContext->appStatus.fogcloudStatus.RecvRomFileSize){
        //no need to update, return size = 0, no need to boot bootloader
        err = kNoErr;
        goto exit;
      }
      
      mico_rtos_lock_mutex(&inContext->flashContentInRam_mutex);
      memset(&inContext->flashContentInRam.bootTable, 0, sizeof(boot_table_t));
      inContext->flashContentInRam.bootTable.length = inContext->appStatus.fogcloudStatus.RecvRomFileSize;
      inContext->flashContentInRam.bootTable.start_address = UPDATE_START_ADDRESS;
      inContext->flashContentInRam.bootTable.type = 'A';
      inContext->flashContentInRam.bootTable.upgrade_type = 'U';
      MICOUpdateConfiguration(inContext);
      
      mico_rtos_unlock_mutex(&inContext->flashContentInRam_mutex);
      inContext->micoStatus.sys_state = eState_Software_Reset;
      require(inContext->micoStatus.sys_state_change_sem, exit);
      mico_rtos_set_semaphore(&inContext->micoStatus.sys_state_change_sem);
    }
    goto exit;
  }
  else{
    return kNotFoundErr;
  };
  
exit:
  if((kNoErr != err) && (fd > 0)){
    //ECS_CreateSimpleHTTPFailedMessage( &httpResponse, &httpResponseLen );
    sprintf(err_msg, "{\"error\": %d}", err);
    ECS_CreateHTTPFailedMessage("500", "FAILED", ECS_kMIMEType_JSON, strlen(err_msg),
                                (uint8_t*)err_msg, strlen(err_msg),
                                &httpResponse, &httpResponseLen );
    require( httpResponse, exit );
    SocketSend( fd, httpResponse, httpResponseLen );
    SocketClose(&fd);
  }
  if(httpResponse) free(httpResponse);
  if(report) json_object_put(report);
  return err;
}
