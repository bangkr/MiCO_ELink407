/**
******************************************************************************
* @file    platform.c 
* @author  William Xu
* @version V1.0.0
* @date    05-May-2014
* @brief   This file provides all MICO Peripherals mapping table and platform
*          specific funcgtions.
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

#include "stdio.h"
#include "string.h"

#include "MICOPlatform.h"
#include "platform.h"
#include "stm32f4xx_platform.h"
#include "platform_common_config.h"
//#include "platform_peripheral.h"
#include "PlatformLogging.h"
#include "spi_flash_platform_interface.h"
#include "wlan_platform_common.h"

#ifdef USE_MiCOKit_EXT
  #include "micokit_ext.h"
#endif

/******************************************************
*                      Macros
******************************************************/

#ifdef __GNUC__
#define WEAK __attribute__ ((weak))
#elif defined ( __IAR_SYSTEMS_ICC__ )
#define WEAK __weak
#endif /* ifdef __GNUC__ */

/******************************************************
*                    Constants
******************************************************/

/******************************************************
*                   Enumerations
******************************************************/

/******************************************************
*                 Type Definitions
******************************************************/

/******************************************************
*                    Structures
******************************************************/

/******************************************************
*               Function Declarations
******************************************************/
extern WEAK void PlatformEasyLinkButtonClickedCallback(void);
extern WEAK void PlatformStandbyButtonClickedCallback(void);
extern WEAK void PlatformEasyLinkButtonLongPressedCallback(void);
extern WEAK void bootloader_start(void);

/******************************************************
*               Variables Definitions
******************************************************/

/* This table maps STM32 pins to GPIO definitions on the schematic
* A full pin definition is provided in <WICED-SDK>/include/platforms/BCM943362WCD4/platform.h
*/

static uint32_t _default_start_time = 0;
static mico_timer_t _button_EL_timer;

const platform_pin_mapping_t gpio_mapping[] =
{
  /* Common GPIOs for internal use */
//  [MICO_GPIO_WLAN_POWERSAVE_CLOCK]    = {WL_32K_OUT_BANK, WL_32K_OUT_PIN, WL_32K_OUT_BANK_CLK},
//  [WL_GPIO0]                          = {GPIOB,  9,  RCC_AHB1Periph_GPIOB},
//  [WL_GPIO1]                          = {GPIOC,  5,  RCC_AHB1Periph_GPIOC},
  [WL_RESET]                          = {GPIOE,  1,  RCC_AHB1Periph_GPIOE},
//  [MICO_SYS_LED]                      = {GPIOB,  0,  RCC_AHB1Periph_GPIOB}, 
//  [MICO_RF_LED]                       = {GPIOA,  4,  RCC_AHB1Periph_GPIOA},
  [BOOT_SEL]                          = {GPIOE,  2,  RCC_AHB1Periph_GPIOE},
//  [MFG_SEL]                           = {GPIOC,  0,  RCC_AHB1Periph_GPIOC},
//  [Standby_SEL]                       = {GPIOA,  1,  RCC_AHB1Periph_GPIOA}, 
  [EasyLink_BUTTON]                   = {GPIOE,  4,  RCC_AHB1Periph_GPIOE}, 
  [STDIO_UART_RX]                     = {GPIOA, 10,  RCC_AHB1Periph_GPIOA},  
  [STDIO_UART_TX]                     = {GPIOA,  9,  RCC_AHB1Periph_GPIOA}, 

  /* GPIOs for external use */
  [MICO_GPIO_0]  = {GPIOA,   9,  RCC_AHB1Periph_GPIOA},
  [MICO_GPIO_1]  = {GPIOA,  10,  RCC_AHB1Periph_GPIOA},
  
  [MICO_GPIO_4]                       = { GPIOA, 15 },   
  [MICO_GPIO_5]                       = { GPIOB,  3 },   
  [MICO_GPIO_6]                       = { GPIOA, 11 },
  [MICO_GPIO_7]                       = { GPIOB,  4 },
  [MICO_GPIO_9]                       = { GPIOA,  4 },
  [MICO_GPIO_10]                      = { GPIOB,  8 },
  [MICO_GPIO_11]                      = { GPIOB,  9 },
  [MICO_GPIO_14]                      = { GPIOC, 13 },
  [MICO_GPIO_21]                      = { GPIOA,  2 }, //UART1_TX_DEBUG
  [MICO_GPIO_22]                      = { GPIOA,  3 }, //UART1_RX_DEBUG
  [MICO_GPIO_23]                      = { GPIOA,  0 }, //EasyLink_BUTTON
  [MICO_GPIO_24]                      = { GPIOA,  1 },
  [MICO_GPIO_25]                      = { GPIOB,  0 }, //spi_flash_spi_pins
  [MICO_GPIO_26]                      = { GPIOB,  1 }, //spi_flash_spi_pins
  [MICO_GPIO_27]                      = { GPIOA, 12 }, //spi_flash_spi_pins
  [MICO_GPIO_28]                      = { GPIOA, 10 }, //spi_flash_spi_pins
  [MICO_GPIO_29]                      = { GPIOA,  5 }, 
  [MICO_GPIO_30]                      = { GPIOB,  2 }, //BOOT
  [MICO_GPIO_31]                      = { GPIOA,  7 },
  [MICO_GPIO_32]                      = { GPIOB, 12 }, //MICO_SYS_LED
  [MICO_GPIO_33]                      = { GPIOB, 13 }, //MFG
};


/* PWM mappings */
const platform_pwm_mapping_t pwm_mappings[] =
{
#if ( MICO_WLAN_POWERSAVE_CLOCK_SOURCE == MICO_WLAN_POWERSAVE_CLOCK_IS_PWM )
  /* Extended PWM for internal use */
  [MICO_PWM_WLAN_POWERSAVE_CLOCK] = {TIM1, 1, RCC_APB2Periph_TIM1, GPIO_AF_TIM1, (platform_pin_mapping_t*)&gpio_mapping[MICO_GPIO_WLAN_POWERSAVE_CLOCK] }, /* or TIM2/Ch2                       */
#endif
  [MICO_PWM_1] = NULL,
  //[MICO_PWM_1]  = {TIM4, 3, RCC_APB1Periph_TIM4, GPIO_AF_TIM4, (platform_pin_mapping_t*)&gpio_mapping[MICO_GPIO_10]},    /* or TIM10/Ch1                       */
  //[MICO_PWM_2]  = {TIM12, 1, RCC_APB1Periph_TIM12, GPIO_AF_TIM12, (platform_pin_mapping_t*)&gpio_mapping[MICO_GPIO_13]}, /* or TIM1/Ch2N                       */
  //[MICO_PWM_3]  = {TIM2, 4, RCC_APB1Periph_TIM2, GPIO_AF_TIM2, (platform_pin_mapping_t*)&gpio_mapping[MICO_GPIO_19]},    
  /* TODO: fill in the other options here ... */
};


const platform_uart_mapping_t uart_mapping[] =
{
  [MICO_UART_1] =
  {
    .usart                        = USART1,
    .gpio_af                      = GPIO_AF_USART1,
    .pin_tx                       = &gpio_mapping[STDIO_UART_TX],
    .pin_rx                       = &gpio_mapping[STDIO_UART_RX],
    .pin_cts                      = NULL,
    .pin_rts                      = NULL,
    .usart_peripheral_clock       = RCC_APB2Periph_USART1,
    .usart_peripheral_clock_func  = RCC_APB2PeriphClockCmd,
    .usart_irq                    = USART1_IRQn,  
    .tx_dma                       = DMA2,
    .tx_dma_stream                = DMA2_Stream7,
    .tx_dma_stream_number         = 7,
    .tx_dma_channel               = DMA_Channel_4,
    .tx_dma_peripheral_clock      = RCC_AHB1Periph_DMA2,
    .tx_dma_peripheral_clock_func = RCC_AHB1PeriphClockCmd,
    .tx_dma_irq                   = DMA2_Stream7_IRQn,
    .rx_dma                       = DMA2,
    .rx_dma_stream                = DMA2_Stream2,
    .rx_dma_stream_number         = 2,
    .rx_dma_channel               = DMA_Channel_4,
    .rx_dma_peripheral_clock      = RCC_AHB1Periph_DMA2,
    .rx_dma_peripheral_clock_func = RCC_AHB1PeriphClockCmd,
    .rx_dma_irq                   = DMA2_Stream2_IRQn,
  },
  [MICO_UART_2] =
  {
    .port                         = USART1,
    .pin_tx                       = &platform_gpio_pins[Arduino_TXD],
    .pin_rx                       = &platform_gpio_pins[Arduino_RXD],
    .pin_cts                      = NULL,
    .pin_rts                      = NULL,
    .tx_dma_config =
    {
      .controller                 = DMA2,
      .stream                     = DMA2_Stream7,
      .channel                    = DMA_Channel_4,
      .irq_vector                 = DMA2_Stream7_IRQn,
      .complete_flags             = DMA_HISR_TCIF7,
      .error_flags                = ( DMA_HISR_TEIF7 | DMA_HISR_FEIF7 ),
    },
    .rx_dma_config =
    {
      .controller                 = DMA2,
      .stream                     = DMA2_Stream2,
      .channel                    = DMA_Channel_4,
      .irq_vector                 = DMA2_Stream2_IRQn,
      .complete_flags             = DMA_LISR_TCIF2,
      .error_flags                = ( DMA_LISR_TEIF2 | DMA_LISR_FEIF2 | DMA_LISR_DMEIF2 ),
    },
  },
};

const platform_i2c_mapping_t i2c_mapping[] =
{
  [MICO_I2C_1] = NULL,
  // [MICO_I2C_1] =
  // {
  //   .i2c = I2C1,
  //   .pin_scl                 = &gpio_mapping[MICO_GPIO_1],
  //   .pin_sda                 = &gpio_mapping[MICO_GPIO_2],
  //   .peripheral_clock_reg    = RCC_APB1Periph_I2C1,
  //   .tx_dma                  = DMA1,
  //   .tx_dma_peripheral_clock = RCC_AHB1Periph_DMA1,
  //   .tx_dma_stream           = DMA1_Stream7,
  //   .rx_dma_stream           = DMA1_Stream5,
  //   .tx_dma_stream_id        = 7,
  //   .rx_dma_stream_id        = 5,
  //   .tx_dma_channel          = DMA_Channel_1,
  //   .rx_dma_channel          = DMA_Channel_1,
  //   .gpio_af                 = GPIO_AF_I2C1
  // },
};


const platform_spi_t platform_spi_peripherals[] =
{
  [MICO_SPI_1]  =
  {
    .port                         = SPI5,
    .gpio_af                      = GPIO_AF6_SPI5,
    .peripheral_clock_reg         = RCC_APB2Periph_SPI5,
    .peripheral_clock_func        = RCC_APB2PeriphClockCmd,
    .pin_mosi                     = &platform_gpio_pins[MICO_GPIO_28],
    .pin_miso                     = &platform_gpio_pins[MICO_GPIO_27],
    .pin_clock                    = &platform_gpio_pins[MICO_GPIO_25],
    .tx_dma = 
    {
      .controller                 = DMA2,
      .stream                     = DMA2_Stream6,
      .channel                    = DMA_Channel_7,
      .irq_vector                 = DMA2_Stream6_IRQn,
      .complete_flags             = DMA_HISR_TCIF6,
      .error_flags                = ( DMA_HISR_TEIF6 | DMA_HISR_FEIF6 ),
    },
    .rx_dma = 
    {
      .controller                 = DMA2,
      .stream                     = DMA2_Stream5,
      .channel                    = DMA_Channel_7,
      .irq_vector                 = DMA2_Stream5_IRQn,
      .complete_flags             = DMA_HISR_TCIF5,
      .error_flags                = ( DMA_HISR_TEIF5 | DMA_HISR_FEIF5 | DMA_HISR_DMEIF5 ),
    },
  }
};

platform_spi_driver_t platform_spi_drivers[MICO_SPI_MAX];



const platform_flash_t platform_flash_peripherals[] =
{
  [MICO_SPI_FLASH] =
  {
    .flash_type                   = FLASH_TYPE_SPI,
    .flash_start_addr             = 0x000000,
    .flash_length                 = 0x200000,
  },
  [MICO_INTERNAL_FLASH] =
  {
    .flash_type                   = FLASH_TYPE_INTERNAL,
    .flash_start_addr             = 0x08000000,
    .flash_length                 = 0x80000,
  },
};

platform_flash_driver_t platform_flash_drivers[MICO_FLASH_MAX];



#if defined ( USE_MICO_SPI_FLASH )
const mico_spi_device_t mico_spi_flash =
{
    .port        = MICO_SPI_1,
    .chip_select = MICO_GPIO_26,
    .speed       = 40000000,
    .mode        = (SPI_CLOCK_RISING_EDGE | SPI_CLOCK_IDLE_HIGH | SPI_USE_DMA | SPI_MSB_FIRST),
    .bits        = 8
};
#endif

const platform_adc_t platform_adc_peripherals[] =
{
  [MICO_ADC_1] = { ADC1, ADC_Channel_4, RCC_APB2Periph_ADC1, 1, (platform_gpio_t*)&platform_gpio_pins[MICO_GPIO_9] },
  [MICO_ADC_2] = { ADC1, ADC_Channel_1, RCC_APB2Periph_ADC1, 1, (platform_gpio_t*)&platform_gpio_pins[MICO_GPIO_24] },
};

/* Wi-Fi control pins. Used by platform/MCU/wlan_platform_common.c
*/
const platform_gpio_t wifi_control_pins[] =
{
  [WIFI_PIN_POWER       ] = { GPIOB, 6 },
};

/* Wi-Fi SDIO bus pins. Used by platform/MCU/STM32F2xx/EMW1062_driver/wlan_SDIO.c */
const platform_gpio_t wifi_sdio_pins[] =
{
#ifdef SDIO_1_BIT
  [WIFI_PIN_SDIO_IRQ    ] = { GPIOA,  8 },
#endif
  [WIFI_PIN_SDIO_CLK    ] = { GPIOB, 15 },
  [WIFI_PIN_SDIO_CMD    ] = { GPIOA,  6 },
  [WIFI_PIN_SDIO_D0     ] = { GPIOB,  7 },
#ifndef SDIO_1_BIT
  [WIFI_PIN_SDIO_D1     ] = { GPIOA,  8 },
  [WIFI_PIN_SDIO_D2     ] = { GPIOA,  9 },
  [WIFI_PIN_SDIO_D3     ] = { GPIOB,  5 },
#endif
};


/******************************************************
*           Interrupt Handler Definitions
******************************************************/

/* USART2 -> MICO_UART_1 */
MICO_RTOS_DEFINE_ISR( USART2_IRQHandler )
{
  platform_uart_irq( &platform_uart_drivers[MICO_UART_1] );
}

MICO_RTOS_DEFINE_ISR( DMA1_Stream6_IRQHandler )
{
  platform_uart_tx_dma_irq( &platform_uart_drivers[MICO_UART_1] );
}

MICO_RTOS_DEFINE_ISR( DMA1_Stream5_IRQHandler )
{
  platform_uart_rx_dma_irq( &platform_uart_drivers[MICO_UART_1] );
}

/* USART1 -> MICO_UART_2 */
MICO_RTOS_DEFINE_ISR( USART1_IRQHandler )
{
  platform_uart_irq( &platform_uart_drivers[MICO_UART_2] );
}

MICO_RTOS_DEFINE_ISR( DMA2_Stream7_IRQHandler )
{
  platform_uart_tx_dma_irq( &platform_uart_drivers[MICO_UART_2] );
}

MICO_RTOS_DEFINE_ISR( DMA2_Stream2_IRQHandler )
{
  platform_uart_rx_dma_irq( &platform_uart_drivers[MICO_UART_2] );
}

void platform_init_peripheral_irq_priorities( void )
{
  /* Interrupt priority setup. Called by WICED/platform/MCU/STM32F2xx/platform_init.c */
  NVIC_SetPriority( RTC_WKUP_IRQn    ,  1 ); /* RTC Wake-up event   */
  NVIC_SetPriority( SDIO_IRQn        ,  2 ); /* WLAN SDIO           */
  NVIC_SetPriority( DMA2_Stream3_IRQn,  3 ); /* WLAN SDIO DMA       */
  NVIC_SetPriority( USART1_IRQn      ,  6 ); /* MICO_UART_1         */
  // NVIC_SetPriority( USART2_IRQn      ,  6 ); /* MICO_UART_2         */
  NVIC_SetPriority( DMA1_Stream6_IRQn,  7 ); /* MICO_UART_1 TX DMA  */
  NVIC_SetPriority( DMA1_Stream5_IRQn,  7 ); /* MICO_UART_1 RX DMA  */
  // NVIC_SetPriority( DMA2_Stream7_IRQn,  7 ); /* MICO_UART_2 TX DMA  */
  // NVIC_SetPriority( DMA2_Stream2_IRQn,  7 ); /* MICO_UART_2 RX DMA  */
  NVIC_SetPriority( EXTI0_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI1_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI2_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI3_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI4_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI9_5_IRQn     , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI15_10_IRQn   , 14 ); /* GPIO                */
}


/******************************************************
*               Function Definitions
******************************************************/

static void _button_EL_irq_handler( void* arg )
{
  (void)(arg);
  int interval = -1;
  
  if ( MicoGpioInputGet( (mico_gpio_t)EasyLink_BUTTON ) == 0 ) {
    _default_start_time = mico_get_time()+1;
    mico_start_timer(&_button_EL_timer);
  } else {
    interval = mico_get_time() + 1 - _default_start_time;
    if ( (_default_start_time != 0) && interval > 50 && interval < RestoreDefault_TimeOut){
      /* EasyLink button clicked once */
      PlatformEasyLinkButtonClickedCallback();
    }
    mico_stop_timer(&_button_EL_timer);
    _default_start_time = 0;
  }
}

static void _button_STANDBY_irq_handler( void* arg )
{
  (void)(arg);
  PlatformStandbyButtonClickedCallback();
}

static void _button_EL_Timeout_handler( void* arg )
{
  (void)(arg);
  _default_start_time = 0;
  PlatformEasyLinkButtonLongPressedCallback();
}

bool watchdog_check_last_reset( void )
{
  if ( RCC->CSR & RCC_CSR_WDGRSTF )
  {
    /* Clear the flag and return */
    RCC->CSR |= RCC_CSR_RMVF;
    return true;
  }
  
  return false;
}

OSStatus mico_platform_init( void )
{
  platform_log( "Platform initialised" );
  
  if ( true == watchdog_check_last_reset() )
  {
    platform_log( "WARNING: Watchdog reset occured previously. Please see watchdog.c for debugging instructions." );
  }
  
  return kNoErr;
}

void init_platform( void )
{
   MicoGpioInitialize( (mico_gpio_t)MICO_SYS_LED, OUTPUT_PUSH_PULL );
   MicoGpioOutputLow( (mico_gpio_t)MICO_SYS_LED );
   MicoGpioInitialize( (mico_gpio_t)MICO_RF_LED, OUTPUT_PUSH_PULL );
   MicoGpioOutputLow( (mico_gpio_t)MICO_RF_LED );
  
   //  Initialise EasyLink buttons
   MicoGpioInitialize( (mico_gpio_t)EasyLink_BUTTON, INPUT_PULL_UP );
   mico_init_timer(&_button_EL_timer, RestoreDefault_TimeOut, _button_EL_Timeout_handler, NULL);
   MicoGpioEnableIRQ( (mico_gpio_t)EasyLink_BUTTON, IRQ_TRIGGER_BOTH_EDGES, _button_EL_irq_handler, NULL );
   
#ifdef USE_MiCOKit_EXT
  MicoGpioInitialize( Arduino_D9, OUTPUT_PUSH_PULL );
  MicoGpioOutputLow( Arduino_D9 );
  
   //  Initialise Standby/wakeup switcher
   MicoGpioInitialize( (mico_gpio_t)Standby_SEL, INPUT_PULL_UP );
   MicoGpioEnableIRQ( (mico_gpio_t)Standby_SEL , IRQ_TRIGGER_FALLING_EDGE, _button_STANDBY_irq_handler, NULL);
   
}

void init_platform_bootloader( void )
{
  MicoGpioInitialize( (mico_gpio_t)MICO_SYS_LED, OUTPUT_PUSH_PULL );
  MicoGpioOutputLow( (mico_gpio_t)MICO_SYS_LED );
  MicoGpioInitialize( (mico_gpio_t)MICO_RF_LED, OUTPUT_PUSH_PULL );
  MicoGpioOutputLow( (mico_gpio_t)MICO_RF_LED );
  
  MicoGpioInitialize((mico_gpio_t)BOOT_SEL, INPUT_PULL_UP);
  MicoGpioInitialize((mico_gpio_t)MFG_SEL, INPUT_HIGH_IMPEDANCE);
  
#ifdef USE_MiCOKit_EXT
  MicoGpioInitialize( Arduino_D9, OUTPUT_PUSH_PULL );
  MicoGpioOutputLow( Arduino_D9 );
  
  rgb_led_init();
  rgb_led_open(0, 0, 0);
#endif
}

void host_platform_reset_wifi( bool reset_asserted )
{
  if ( reset_asserted == true )
  {
    MicoGpioOutputLow( (mico_gpio_t)WL_RESET );  
  }
  else
  {
    MicoGpioOutputHigh( (mico_gpio_t)WL_RESET ); 
  }
}

void host_platform_power_wifi( bool power_enabled )
{
  if ( power_enabled == true )
  {
    MicoGpioOutputLow( (mico_gpio_t)WL_REG );  
  }
  else
  {
    MicoGpioOutputHigh( (mico_gpio_t)WL_REG ); 
  }
}






void MicoSysLed(bool onoff)
{
    if (onoff) {
        MicoGpioOutputHigh( (mico_gpio_t)MICO_SYS_LED );
    } else {
        MicoGpioOutputLow( (mico_gpio_t)MICO_SYS_LED );
    }
}

// Only one led on base board, so use system led as RF led.
void MicoRfLed(bool onoff)
{
    if (onoff) {
        MicoGpioOutputHigh( (mico_gpio_t)MICO_RF_LED );
    } else {
        MicoGpioOutputLow( (mico_gpio_t)MICO_RF_LED );
    }
}

bool MicoShouldEnterMFGMode(void)
{
  if( MicoGpioInputGet((mico_gpio_t)BOOT_SEL)==false && MicoGpioInputGet((mico_gpio_t)MFG_SEL)==false ){
    return true;
  }
  else{
    return false;
  }
}

// add test mode for MiCOKit-EXT board,check Arduino_D5 pin when system reboot
bool MicoShouldEnterTestMode(void)
{
  if( MicoGpioInputGet((mico_gpio_t)Arduino_D5)==false ){
    return true;
  }
  else{
    return false;
  }
}

// bootloader mode: SW1=ON, SW2=OFF
bool MicoShouldEnterBootloader(void)
{
  if(MicoGpioInputGet((mico_gpio_t)BOOT_SEL)==false && MicoGpioInputGet((mico_gpio_t)MFG_SEL)==true)
    return true;
  else
    return false;
}

