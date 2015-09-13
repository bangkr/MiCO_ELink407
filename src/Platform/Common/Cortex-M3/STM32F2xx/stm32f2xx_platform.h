/**
******************************************************************************
* @file    stm32f2xx_platform.h 
* @author  William Xu
* @version V1.0.0
* @date    05-May-2014
* @brief   This file provide all the headers of functions for stm32f2xx platform
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

#pragma once

#include "stm32f2xx.h"
#include "stm32f2xx_dma.h"
#include "stm32f2xx_adc.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_tim.h"
#include "stm32f2xx_rtc.h"
#include "stm32f2xx_pwr.h"
#include "stm32f2xx_rng.h"

#include "MicoRtos.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef USART_TypeDef      usart_registers_t;
typedef DMA_TypeDef        dma_registers_t;
typedef DMA_Stream_TypeDef dma_stream_registers_t;
typedef uint32_t           dma_channel_t;
typedef IRQn_Type          irq_vector_t;
typedef FunctionalState    functional_state_t;
typedef uint32_t           peripheral_clock_t;
typedef void (*peripheral_clock_func_t)(peripheral_clock_t clock, functional_state_t state );

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    GPIO_TypeDef* bank;
    uint8_t       number;
    uint32_t      peripheral_clock;
} platform_pin_mapping_t;

typedef struct
{
    ADC_TypeDef*            adc;
    uint8_t                 channel;
    uint32_t                adc_peripheral_clock;
    uint8_t                 rank;
    platform_pin_mapping_t* pin;
} platform_adc_mapping_t;

typedef struct
{
    TIM_TypeDef*            tim;
    uint8_t                 channel;
    uint32_t                tim_peripheral_clock;
    uint8_t                 gpio_af;
    platform_pin_mapping_t* pin;
} platform_pwm_mapping_t;


/* DMA can be enabled by setting SPI_USE_DMA */
typedef struct
{
    SPI_TypeDef*                  spi_regs;
    uint8_t                       gpio_af;
    uint32_t                      peripheral_clock_reg;
    peripheral_clock_func_t       peripheral_clock_func;
    const platform_pin_mapping_t* pin_mosi;
    const platform_pin_mapping_t* pin_miso;
    const platform_pin_mapping_t* pin_clock;
    DMA_Stream_TypeDef*           tx_dma_stream;
    DMA_Stream_TypeDef*           rx_dma_stream;
    uint32_t                      tx_dma_channel;
    uint32_t                      rx_dma_channel;
    uint8_t                       tx_dma_stream_number;
    uint8_t                       rx_dma_stream_number;
} platform_spi_mapping_t;

typedef void (* wakeup_irq_handler_t)(void *arg);

typedef struct
{
    USART_TypeDef*                usart;
    uint8_t                       gpio_af;
    const platform_pin_mapping_t* pin_tx;
    const platform_pin_mapping_t* pin_rx;
    const platform_pin_mapping_t* pin_cts;
    const platform_pin_mapping_t* pin_rts;
    peripheral_clock_t            usart_peripheral_clock;
    peripheral_clock_func_t       usart_peripheral_clock_func;
    irq_vector_t                  usart_irq;
    dma_registers_t*              tx_dma;
    dma_stream_registers_t*       tx_dma_stream;
    uint8_t                       tx_dma_stream_number;
    dma_channel_t                 tx_dma_channel;
    peripheral_clock_t            tx_dma_peripheral_clock;
    peripheral_clock_func_t       tx_dma_peripheral_clock_func;
    irq_vector_t                  tx_dma_irq;
    dma_registers_t*              rx_dma;
    dma_stream_registers_t*       rx_dma_stream;
    uint8_t                       rx_dma_stream_number;
    dma_channel_t                 rx_dma_channel;
    peripheral_clock_t            rx_dma_peripheral_clock;
    peripheral_clock_func_t       rx_dma_peripheral_clock_func;
    irq_vector_t                  rx_dma_irq;
} platform_uart_mapping_t;

typedef struct
{
    I2C_TypeDef*            i2c;
    const platform_pin_mapping_t* pin_scl;
    const platform_pin_mapping_t* pin_sda;
    uint32_t                peripheral_clock_reg;
    dma_registers_t*              tx_dma;
    peripheral_clock_t            tx_dma_peripheral_clock;
    DMA_Stream_TypeDef* tx_dma_stream;
    DMA_Stream_TypeDef* rx_dma_stream;
    int                 tx_dma_stream_id;
    int                 rx_dma_stream_id;
    uint32_t            tx_dma_channel;
    uint32_t            rx_dma_channel;
    uint32_t                      gpio_af;
} platform_i2c_mapping_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

extern const platform_pin_mapping_t  gpio_mapping[];
extern const platform_adc_mapping_t  adc_mapping[];
extern const platform_pwm_mapping_t  pwm_mappings[];
extern const platform_spi_mapping_t  spi_mapping[];
extern const platform_uart_mapping_t uart_mapping[];
extern const platform_i2c_mapping_t  i2c_mapping[];


/******************************************************
 *               Function Declarations
 ******************************************************/
