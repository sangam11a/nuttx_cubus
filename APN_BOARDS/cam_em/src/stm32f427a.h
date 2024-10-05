/****************************************************************************
 * boards/arm/stm32/stm32f427a-minimal/src/stm32f427a.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_CUBUS_V1_SRC_STM32F427A_H
#define __BOARDS_ARM_CUBUS_V1_SRC_STM32F427A_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>


#include <arch/stm32/chip.h>

#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_PROC       1
#define HAVE_USBDEV     1
#define HAVE_USBMONITOR 1

/* Can't support USB host or device features if USB OTG HS is not enabled */

#ifndef CONFIG_STM32_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBFS
#endif

/* Can't support USB device monitor if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

/* Can't support USB host is USB host is not enabled */


/* Check if we should enable the USB monitor before starting NSH */

#ifndef CONFIG_USBMONITOR
#  undef HAVE_USBMONITOR
#endif

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif



#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef HAVE_USBMONITOR
#endif

/* Check if we have the procfs file system */

#if !defined(CONFIG_FS_PROCFS)
#  undef HAVE_PROC
#endif

#if defined(HAVE_PROC) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No procfs support
#  undef HAVE_PROC
#endif

/* How many SPI modules does this chip support? */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#  undef CONFIG_STM32_SPI4
#  undef CONFIG_STM32_SPI5
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#  undef CONFIG_STM32_SPI4
#  undef CONFIG_STM32_SPI5
#elif STM32_NSPI < 3
#  undef CONFIG_STM32_SPI3
#  undef CONFIG_STM32_SPI4
#  undef CONFIG_STM32_SPI5
#elif STM32_NSPI < 4
#  undef CONFIG_STM32_SPI4
#  undef CONFIG_STM32_SPI5
#elif STM32_NSPI < 5
#  undef CONFIG_STM32_SPI5
#endif

#ifdef CONFIG_STM32_ADC1
#define ADC1_PATH   "/dev/adc0"
#endif  //CONFIG_STM32_ADC1

#ifdef CONFIG_STM32_ADC2
#define ADC2_PATH   "/dev/adc1"
#endif

#ifdef CONFIG_STM32_ADC3
#define ADC3_PATH   "dev/adc2"
#endif  // CONFIG_STM32_ADC3

#define EXT_ADC_PATH    "/dev/ext_adc0"
#define MAG_1_PATH      "/dev/mag0"


/* STM32F427 minimal board GPIOs ************************************************/

#define GPIO_SFM_CS			        (GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN11)

#define GPIO_OCP_EN			        (GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN4)


/* PWM
 *
 * The STM32F427 minimal board has no real on-board PWM devices, but the board
 * can be configured to output a pulse train using TIM4 CH2 on PD13.
 */

#define STM32F427A_PWMTIMER   4
#define STM32F427A_PWMCHANNEL 2





/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************

 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f429i-disco
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in inialization to setup USB-
 *   related GPIO pins for the STM32F427 Cubus board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC device.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CUBUS_V1_SRC_STM32F427A_H */
