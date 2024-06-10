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

#ifdef CONFIG_SENSORS_LIS3MDL
static int stm32_attach_irq(const struct lis3mdl_config_s *lower,
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
  .dev.attach = stm32_attach_irq,
  .dev.spi_devid = SPIDEV_USER(0),
  .handler = NULL,
  .intcfg = GPIO_LIS3MDL_INT,
};
#endif

#ifdef CONFIG_ADC_ADS7953
static struct adc_priv_s adc0 =
{
  .dev.spi = NULL,
  .dev.spi_devid = SPIDEV_USER(1),
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
  ret = ads7953_register(EXT_ADC_PATH, adc0.dev.spi, adc0.dev.spi_devid);
  if (ret < 0)
  {
    syslog(LOG_ERR,"[BRINGUP] ads7953 register failed.\n");
  } else {
    syslog(LOG_INFO,"[BRINGUP] Registered ads7953.\n");
  }
#endif  // CONFIG_ADC_ADS7953

#if defined(CONFIG_MTD) && defined(CONFIG_MTD_PROGMEM)
  mtd = progmem_initialize();
  if (mtd == NULL)
    {
      syslog(LOG_ERR, "ERROR: progmem_initialize\n");
    }

  ret = register_mtddriver("/dev/flash", mtd, 0, mtd);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: register_mtddriver() failed: %d\n", ret);
    }

#endif

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

  
  /* Now bind the SPI interface to the SST25F064 SPI FLASH driver.  This
   * is a FLASH device that has been added external to the board (i.e.
   * the board does not ship from STM with any on-board FLASH.
   */

// #if defined(CONFIG_MTD) && defined(CONFIG_MTD_MT25QL)
//   syslog(LOG_INFO, "Bind SPI to the SPI flash driver\n");

//   mtd = mt25ql_initialize(spi3);
//   if (!mtd)
//     {
//       syslog(LOG_ERR, "ERROR: Failed to bind SPI port 3 to the SPI FLASH"
//                       " driver\n");
//     }
//   else
//     {
//       syslog(LOG_INFO, "Successfully bound SPI port 3 to the SPI FLASH"
//                        " driver\n");

//       /* Get the geometry of the FLASH device */

//       ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY,
//                        (unsigned long)((uintptr_t)&geo));
//       if (ret < 0)
//         {
//           syslog("ERROR: mtd->ioctl failed: %d\n", ret);
//         }

// #ifdef CONFIG_STM32F427A_FLASH_PART
//         {
//           int partno;
//           int partsize;
//           int partoffset;
//           int partszbytes;
//           int erasesize;
//           const char *partstring = CONFIG_STM32F427A_FLASH_PART_LIST;
//           const char *ptr;
//           struct mtd_dev_s *mtd_part;
//           char  partref[16];

//           /* Now create a partition on the FLASH device */

//           partno = 0;
//           ptr = partstring;
//           partoffset = 0;

//           /* Get the Flash erase size */

//           erasesize = geo.erasesize;

//           while (*ptr != '\0')
//             {
//               /* Get the partition size */

//               partsize = atoi(ptr);
//               partszbytes = (partsize << 10); /* partsize is defined in KB */

//               /* Check if partition size is bigger then erase block */

//               if (partszbytes < erasesize)
//                 {
//                   syslog("ERROR: Partition size is lesser than erasesize!\n");
//                   return -1;
//                 }

//               /* Check if partition size is multiple of erase block */

//               if ((partszbytes % erasesize) != 0)
//                 {
//                   syslog("ERROR: Partition size is not multiple of"
//                        " erasesize!\n");
//                   return -1;
//                 }

//               mtd_part    = mtd_partition(mtd, partoffset,
//                                           partszbytes / erasesize);
//               partoffset += partszbytes / erasesize;

// #ifdef CONFIG_STM32F427A_FLASH_CONFIG_PART
//               /* Test if this is the config partition */

//               if (CONFIG_STM32F427A_FLASH_CONFIG_PART_NUMBER == partno)
//                 {
//                   /* Register the partition as the config device */

//                   mtdconfig_register(mtd_part);
//                 }
//               else
// #endif
//                 {
//                   /* Now initialize a SMART Flash block device and bind it
//                    * to the MTD device.
//                    */

// #if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
//                   snprintf(partref, sizeof(partref), "p%d", partno);
//                   smart_initialize(CONFIG_STM32F427A_FLASH_MINOR,
//                                    mtd_part, partref);
// #endif
//                 }

// #if defined(CONFIG_MTD_PARTITION_NAMES)
//               /* Set the partition name */

//               if (mtd_part == NULL)
//                 {
//                   ferr("ERROR: failed to create partition %s\n", partname);
//                   return -1;
//                 }

//               mtd_setpartitionname(mtd_part, partname);

//               /* Now skip to next name.  We don't need to split the string
//                * here because the MTD partition logic will only display names
//                * up to the comma, thus allowing us to use a single static
//                * name in the code.
//                */

//               while (*partname != ',' && *partname != '\0')
//                 {
//                   /* Skip to next ',' */

//                   partname++;
//                 }

//               if (*partname == ',')
//                 {
//                   partname++;
//                 }
// #endif

//               /* Update the pointer to point to the next size in the list */

//               while ((*ptr >= '0') && (*ptr <= '9'))
//                 {
//                   ptr++;
//                 }

//               if (*ptr == ',')
//                 {
//                   ptr++;
//                 }

//               /* Increment the part number */

//               partno++;
//             }
//         }
// #endif /* CONFIG_STM32F427A_FLASH_PART */
//     }

// #endif /* CONFIG_MTD */
#endif /* CONFIG_STM32_SPI3 */

// #if defined(CONFIG_RAMMTD) && defined(CONFIG_STM32F427A_RAMMTD)
//   /* Create a RAM MTD device if configured */

//     {
//       uint8_t *start =
//           kmm_malloc(CONFIG_STM32F427A_RAMMTD_SIZE * 1024);
//       mtd = rammtd_initialize(start,
//                               CONFIG_STM32F427A_RAMMTD_SIZE * 1024);
//       mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);

//       /* Now initialize a SMART Flash block device and bind it to the MTD
//        * device
//        */

// #if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
//       smart_initialize(CONFIG_STM32F427A_RAMMTD_MINOR, mtd, NULL);
// #endif
//     }

// #endif /* CONFIG_RAMMTD && CONFIG_STM32F427A_RAMMTD */

#ifdef CONFIG_SENSORS_LIS3MDL
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

  ret = lis3mdl_register("/dev/mag0", spi5, &mag0.dev);
  if (ret < 0)
  {
    syslog(LOG_INFO,"[BRING_UP] Error: Failed to register LIS3MDL driver.\n");
  } else {
    syslog(LOG_INFO,"[BRING_UP] LIS3MDL registered on SPI 5.\n");
  }
#endif  // CONFIG_SENSORS_LIS3MDL

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
