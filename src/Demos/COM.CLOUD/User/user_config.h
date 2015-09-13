/**
  ******************************************************************************
  * @file    user_config.h 
  * @author  Eshen Wang
  * @version V1.0.0
  * @date    17-Mar-2015
  * @brief   This file contains user configuration.
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

#ifndef __USER_CONFIG_H_
#define __USER_CONFIG_H_

#include "platform_common_config.h"
#include "platform.h"


/*******************************************************************************
 *                              APP INFO
 ******************************************************************************/
/* product type
 * Replace them with your own product id/secure from fogcloud.io 
 */
//#ifdef 1//MICOKIT_3288
  #define PRODUCT_ID                      "01799d5b"
  #define PRODUCT_KEY                     "22204e06-4db5-4cf3-9ac3-902e0dea70cd"
//#elif MICOKIT_3165
//  #define PRODUCT_ID                      "f4680913"
//  #define PRODUCT_KEY                     "c0558531-fd8b-4d96-a78f-28d3bc5ebda0"
//#else

//#endif

/* Application info */
#define SERIAL_NUMBER                      "1507241538"
#define FIRMWARE_REVISION                  HARDWARE_REVISION"@"SERIAL_NUMBER

#define DEFAULT_ROM_VERSION                FIRMWARE_REVISION
#define DEFAULT_DEVICE_NAME                MODEL   // device name upload to cloud defined in platform_config.h

#define APP_INFO                           MODEL" Enjoy Demo based on MICO OS, fw version: "FIRMWARE_REVISION","
#define PROTOCOL                           "com.mxchip.micokit"


/*******************************************************************************
 *                             CONNECTING
 ******************************************************************************/
/* Wi-Fi configuration mode */
#define MICO_CONFIG_MODE                 CONFIG_MODE_EASYLINK  // now both airkiss && easylink are supportted with this macro
//CONFIG_MODE_SOFT_AP//
/* MICO cloud service type */
#define MICO_CLOUD_TYPE                  CLOUD_FOGCLOUD

// disalbe FogCloud OTA check when system start
#define DISABLE_FOGCLOUD_OTA_CHECK


/*******************************************************************************
 *                             RESOURCES
 ******************************************************************************/
#define STACK_SIZE_USER_MAIN_THREAD         0x800
#define STACK_SIZE_USER_MSG_HANDLER_THREAD  0x800
#define STACK_SIZE_NOTIFY_THREAD            0x800
#define MICO_PROPERTIES_NOTIFY_INTERVAL_MS  2000


/*User provided configurations*/
#define CONFIGURATION_VERSION               0x00000004 // if your configuration is changed, update this number
   
#endif  // __USER_CONFIG_H_
