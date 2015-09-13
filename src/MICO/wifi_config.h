/**
******************************************************************************
* @file    wifi_config.h 
* @author  William Xu
* @version V1.0.0
* @date    05-May-2014
* @brief   This file provide header file for the WifiConfig_ && airkiss function.
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

#ifndef __WIFI_CONFIG_H
#define __WIFI_CONFIG_H

#include "MICODefine.h"

#define WifiConfig_TimeOut                60000 /**< WifiConfig_ timeout 60 seconds. */

#define WifiConfig_ConnectWlan_Timeout    20000 /**< Connect to wlan after configured by WifiConfig.
                                                   Restart WifiConfig after timeout: 20 seconds. */										   

OSStatus startWifiConfig( mico_Context_t * const inContext);

OSStatus stopWifiConfig( mico_Context_t * const inContext);



#endif


