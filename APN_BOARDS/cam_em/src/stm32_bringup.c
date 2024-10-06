/****************************************************************************
 * boards/arm/stm32/stm32f427a-minimal/src/stm32_bringup.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <string.h>

#include <nuttx/spi/spi.h>
#include <stm32_spi.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>

#if defined(CONFIG_MTD_MT25QL) || defined(CONFIG_MTD_PROGMEM)
#include <nuttx/mtd/mtd.h>
#include "cubus_mtd.h"
#endif

#ifndef CONFIG_STM32F427A_FLASH_MINOR
#define CONFIG_STM32F427A_FLASH_MINOR 0
#endif

#ifdef CONFIG_STM32F427A_FLASH_CONFIG_PART
#ifdef CONFIG_PLATFORM_CONFIGDATA
#include <nuttx/mtd/configdata.h>
#endif
#endif

#ifdef CONFIG_SENSORS_LIS3MDL
// #define CONFIG_SENSORS_LIS3MDL_SPI CONFIG_SENSORS_LIS3MDL
#include <nuttx/sensors/lis3mdl.h>
#endif

#ifdef CONFIG_SENSORS_MPU60X0
#include <nuttx/sensors/mpu60x0.h>
#endif

#ifdef CONFIG_ADC_ADS7953
#include <nuttx/analog/ads7953.h>
#include "../../../../drivers/analog/ads7953.c"
#endif

#include "stm32.h"
#include "stm32f427a.h"

#ifdef CONFIG_STM32_OWN_LED
#include "stm32_own_led.h"
#endif

#if defined(CONFIG_STM32_IWDG)
// #include <nuttx/timers/watchdog.h>
#include <nuttx/wdog.h>
// #include <nuttx/arch/arm/src/stm32/stm32_wdg.h>
#endif

#if defined(CONFIG_STM32_SPI2)
struct spi_dev_s *spi2;
#endif

#if defined(CONFIG_STM32_SPI3)
struct spi_dev_s *spi3;
#endif

#if defined(CONFIG_STM32_SPI4)
struct spi_dev_s *spi4;
#endif

#if defined(CONFIG_STM32_SPI5)
struct spi_dev_s *spi5;
#endif

#ifdef CONFIG_SENSORS_LIS3MDL
typedef struct mag_priv_s
{
  struct lis3mdl_config_s dev;
  xcpt_t handler;
  void *arg;
  uint32_t intcfg;
} mag_priv_s;
#endif

struct mtd_dev_s *mtd;

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the MRF24J40 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   irq_attach       - Attach the MRF24J40 interrupt handler to the GPIO
 *                      interrupt
 *   irq_enable       - Enable or disable the GPIO interrupt
 */


/****************************************************************************
 * Public Functions
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

int stm32_bringup(void)
{

  int ret;
  stm32_gpiowrite(GPIO_OCP_EN, false);

  sleep(1);
  // stm32_gpiowrite(GPIO_OCP_EN, true);



  /* Configure SPI-based devices */


// #ifdef CONFIG_MTD_MT25QL

//   /* Init SPI Bus again */

//   spi2 = stm32_spibus_initialize(2);
//   if (!spi2)
//   {
//     syslog(LOG_ERR, "[BRINGUP] Failed to initialize SPI Port 2.\n");
//   }
//   else
//   {
//     syslog(LOG_INFO, "[BRINGUP] Successfully Initalized SPI Port 2.\n");

//     SPI_SETFREQUENCY(spi2, 1000000);
//     SPI_SETBITS(spi2, 8);
//     SPI_SETMODE(spi2, SPIDEV_MODE0);
//   }
//   cubus_mft_configure(board_get_manifest());
 
// #endif // CONFIG_MTD_MT25QL


#ifdef CONFIG_STM32_OWN_LED
  printf("External gpio driver initializing...\n");
  int retval = etx_gpio_driver_init();
  if (retval == -1)
  {
    printf("error on initializing gpio driver..\n");
  }
  else
  {
    printf("Initialized gpio driver successfully");
  }
#endif
  UNUSED(ret);

// #if defined(CONFIG_MTD) && defined(CONFIG_MTD_PROGMEM)
//   mtd = progmem_initialize();
//   if (mtd == NULL)
//   {
//     syslog(LOG_ERR, "ERROR: progmem_initialize\n");
//     printf("[BRINGUP: PROGMEM] error initializing progmem\n");
//   }
//   else
//   {
//     syslog(LOG_INFO, "INFO: Initialized progmem successfully: \n");
//     printf("[BRINGUP: PROGMEM] Initialized progmem sucessfully...\r\n");
//   }

//   ret = register_mtddriver("/dev/intflash", mtd, 0, mtd);
//   if (ret < 0)
//   {
//     syslog(LOG_ERR, "ERROR: register_mtddriver() failed: %d\n", ret);
//     printf("[BRINGUP: PROGMEM] Error registering mtd driver");
//   }
//   else
//   {
//     syslog(LOG_INFO, "INFO: registered mtd driver successfully \n");
//     printf("[BRINGUP: PROGMEM] Registerd internal flash mtd driver successfullyy.....\r\n");
//   }
// #endif


#ifdef CONFIG_STM32_IWDG
  // struct watchdog_lowerhalf_s lower;

  //   / Allocate the lower-half data structure /
  //   lower = (FAR struct watchdog_lowerhalf_s)kmm_zalloc(sizeof(struct watchdog_lowerhalf_s));
  //   if (!lower)
  //   {
  //     return -ENOMEM;
  //   }
  //     struct watchdog_ops_s g_my_watchdog_ops;
  //   /* Initialize the lower-half structure */
  //   lower->ops = &g_my_watchdog_ops;
  // watchdog_register("/dev/watchdog0", &lower);

  stm32_iwdginitialize("/dev/iwdg0", 20000000);
#endif

// stm32_serial_dma_setup();
// stm32_serial_dma_initialize();
// write();
#ifdef CONFIG_STM32_WWDG
  stm32_wwdginitialize("/dev/wwdg0");
#endif
  return 0;
}
