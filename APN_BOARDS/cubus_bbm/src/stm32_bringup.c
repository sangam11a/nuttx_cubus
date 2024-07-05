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
#  include <nuttx/mtd/mtd.h>
#  include "cubus_mtd.h"
#endif

#ifndef CONFIG_STM32F427A_FLASH_MINOR
#define CONFIG_STM32F427A_FLASH_MINOR 0
#endif

#ifdef CONFIG_STM32F427A_FLASH_CONFIG_PART
#ifdef CONFIG_PLATFORM_CONFIGDATA
#  include <nuttx/mtd/configdata.h>
#endif
#endif

#ifdef CONFIG_SENSORS_LIS3MDL
#include <nuttx/sensors/lis3mdl.h>
#endif

#ifdef CONFIG_SENSORS_MPU60X0
#include <nuttx/sensors/mpu60x0.h>
#endif

// #ifdef CONFIG_ADC_ADS7953
#include <nuttx/analog/ads7953.h>
#include "../../../../drivers/analog/ads7953.c"
// #endif

#include "stm32.h"
#include "stm32f427a.h"

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
};
#endif

#ifdef CONFIG_ADC_ADS7953

typedef struct adc_priv_s
{
  struct ads7953_config_s dev;
  xcpt_t handler;
  void *arg;
  uint32_t intcfg;
};
#endif

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

#ifdef CONFIG_SENSORS_LIS3MDL
static int stm32_attach_mag_irq(const struct lis3mdl_config_s *lower,
                            xcpt_t handler, void *arg)
{
  struct mag_priv_s *priv = (struct mag_priv_s *)lower;

  DEBUGASSERT(priv != NULL);

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  priv->arg     = arg;
  return OK;
}

static struct mag_priv_s mag0 =
{
  .dev.attach = stm32_attach_mag_irq,
  .dev.spi_devid = SPIDEV_USER(0),
  .dev.irq  = 0,
  .handler = NULL,
  .intcfg = GPIO_LIS3MDL_DRDY,
};
#endif


#ifdef CONFIG_SENSORS_MPU60X0
struct mpu_config_s mpu0 =
{
  .spi_devid = SPIDEV_IMU(0),
};
#endif

#ifdef CONFIG_ADC_ADS7953
static struct adc_priv_s adc0 =
{
  .dev.spi = NULL,
  .dev.spi_devid = SPIDEV_USER(1),
  .handler = NULL,
  .intcfg = NULL,
};
#endif

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

  /* Configure SPI-based devices */
#ifdef CONFIG_STM32_SPI2

#ifdef CONFIG_ADC_ADS7953

  /* Init SPI Bus again */

  spi2 = stm32_spibus_initialize(2);
  
  if (!spi2)
  {
    syslog(LOG_ERR,"[BRINGUP] Failed to initialize SPI Port 2.\n");
  } else {
    syslog(LOG_INFO,"[BRINGUP] Successfully Initalized SPI Port 2.\n");
    adc0.dev.spi = spi2;
    SPI_SETFREQUENCY(spi2, 1000000);
    SPI_SETBITS(spi2, 8);
    SPI_SETMODE(spi2, SPIDEV_MODE0);
  }

  ret = ads7953_register(CONFIG_EADC0_PATH, adc0.dev.spi, adc0.dev.spi_devid);

  if (ret < 0)
  {
    syslog(LOG_ERR,"[BRINGUP] ads7953 register failed.\n");
  } else {
    syslog(LOG_INFO,"[BRINGUP] Registered ads7953.\n");
  }

#endif  // CONFIG_ADC_ADS7953
#endif  // CONFIG_STM32_SPI2

#ifdef CONFIG_STM32_SPI3
  /* Get the SPI port */

  syslog(LOG_INFO, "Initializing SPI port 3\n");
  spi3 = stm32_spibus_initialize(3);
  if (!spi3)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port 3\n");
    } else {
      syslog(LOG_INFO, "Successfully initialized SPI port 3\n");
    }

  cubus_mft_configure(board_get_manifest());

#endif /* CONFIG_STM32_SPI3 */

#ifdef CONFIG_STM32_SPI5
  spi5 = stm32_spibus_initialize(5);
  if (!spi5)
  {
    syslog(LOG_ERR,"[BRING_UP] ERROR: Failed to Initialize SPI 5 bus.\n");
  } else {
    syslog(LOG_INFO,"[BRING_UP] Initialized bus on SPI port 5.\n");

    SPI_SETFREQUENCY(spi5, 1000000);
    SPI_SETBITS(spi5, 8);
    SPI_SETMODE(spi5, SPIDEV_MODE0);
  }

#ifdef CONFIG_SENSORS_LIS3MDL
  ret = lis3mdl_register(0, spi5, &mag0.dev);
  // ret - lis3mdl_activate()
  if (ret < 0)
  {
    syslog(LOG_INFO,"[BRING_UP] Error: Failed to register LIS3MDL driver.\n");
  } else {
    syslog(LOG_INFO,"[BRING_UP] LIS3MDL registered on SPI 5.\n");
  }
#endif  // CONFIG_SENSORS_LIS3MDL

#ifdef CONFIG_SENSORS_MPU60X0
  mpu0.spi = spi5;
  ret = mpu60x0_register(CONFIG_IMU0_PATH, &mpu0);
  if (ret < 0)
  {
    syslog(LOG_ERR, "[bringup] Failed to initialize MPU6500\n");
  } else {
    syslog(LOG_INFO, "[bringup] Initialized MPU6500.\n");
  }
  
#endif  // CONFIG_SENSORS_MP60X0

#endif  // CONFIG_STM32_SPI5

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC device. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup() failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;

}
