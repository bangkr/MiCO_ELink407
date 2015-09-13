/**
******************************************************************************
* @file    fogcloud.h 
* @author  Eshen Wang
* @version V1.0.0
* @date    17-Mar-2015
* @brief   This header contains the low level interfaces for fogcloud. 
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


#ifndef __FOGCLOUD_INTERFACES_H_
#define __FOGCLOUD_INTERFACES_H_


#include "MICODefine.h"
#include "FogCloudServiceDef.h"

/*******************************************************************************
 *                                DEFINES
 ******************************************************************************/


/*******************************************************************************
 *                               INTERFACES
 ******************************************************************************/
//common interfaces
OSStatus fogCloudInit(mico_Context_t* const inContext);
OSStatus fogCloudStart(mico_Context_t* const inContext);
easycloud_service_state_t fogCloudGetState(void);
OSStatus fogCloudSend(unsigned char *inBuf, unsigned int inBufLen);
OSStatus fogCloudSendto(const char* topic, 
                        unsigned char *inBuf, unsigned int inBufLen);
// send to sub-level, topic "device_id/out/<level>"
OSStatus fogCloudSendtoChannel(const char* channel, 
                               unsigned char *inBuf, unsigned int inBufLen);

// cloud specifical interfaces
OSStatus fogCloudDevActivate(mico_Context_t* const inContext,
                             MVDActivateRequestData_t devActivateReqData);
OSStatus fogCloudDevAuthorize(mico_Context_t* const inContext,
                              MVDAuthorizeRequestData_t devAuthorizeReqData);

OSStatus fogCloudDevFirmwareUpdate(mico_Context_t* const inContext,
                                   MVDOTARequestData_t devOTARequestData);
OSStatus fogCloudResetCloudDevInfo(mico_Context_t* const inContext,
                                   MVDResetRequestData_t devResetRequestData);

OSStatus fogCloudStop(mico_Context_t* const inContext);
OSStatus fogCloudDeinit(mico_Context_t* const inContext);
OSStatus fogCloudPrintVersion(void);


#endif  // __FOGCLOUD_INTERFACES_H_
