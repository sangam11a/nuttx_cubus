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

#ifdef CONFIG_ADC_ADS7953

typedef struct adc_priv_s
{
  struct ads7953_config_s dev;
  xcpt_t handler;
  void *arg;
  uint32_t intcfg;
} adc_priv_s;
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
static int stm32_attach_mag_irq(const struct lis3mdl_config_s *lower,
                                xcpt_t handler, void *arg)
{
  struct mag_priv_s *priv = (struct mag_priv_s *)lower;

  DEBUGASSERT(priv != NULL);

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  priv->arg = arg;
  return OK;
}

static struct mag_priv_s mag0 =
    {
        .dev.attach = stm32_attach_mag_irq,
        .dev.spi_devid = SPIDEV_USER(0),
        .dev.irq = 0,
        .handler = NULL,
        .intcfg = GPIO_LIS3MDL_DRDY,
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

// TODO:REMOVE later
stm32_gpiowrite(GPIO_3V3_COM_EN,true);
  int ret;

  /* Configure SPI-based devices */

#ifdef CONFIG_ADC_ADS7953

  /* Init SPI Bus again */

  spi2 = stm32_spibus_initialize(2);
  if (!spi2)
  {
    syslog(LOG_ERR, "[BRINGUP] Failed to initialize SPI Port 2.\n");
  }
  else
  {
    syslog(LOG_INFO, "[BRINGUP] Successfully Initalized SPI Port 2.\n");
    adc0.dev.spi = spi2;
    SPI_SETFREQUENCY(spi2, 1000000);
    SPI_SETBITS(spi2, 8);
    SPI_SETMODE(spi2, SPIDEV_MODE0);
  }
  ret = ads7953_register(EXT_ADC_PATH, adc0.dev.spi, adc0.dev.spi_devid);
  if (ret < 0)
  {
    syslog(LOG_ERR, "[BRINGUP] ads7953 register failed.\n");
  }
  else
  {
    syslog(LOG_INFO, "[BRINGUP] Registered ads7953.\n");
  }
#endif // CONFIG_ADC_ADS7953

#ifdef CONFIG_STM32_SPI3
  /* Get the SPI port */

  syslog(LOG_INFO, "Initializing SPI port 3\n");
  spi3 = stm32_spibus_initialize(3);
  if (!spi3)
  {
    syslog(LOG_ERR, "ERROR: Failed to initialize SPI port 3\n");
  }
  else
  {
    syslog(LOG_INFO, "Successfully initialized SPI port 3\n");
  }
  stm32_gpiowrite(GPIO_MUX_EN, false);
  stm32_gpiowrite(GPIO_SFM_CS, false);
  stm32_gpiowrite(GPIO_SFM_MODE, false);

  cubus_mft_configure(board_get_manifest());
  // stm32_gpiowrite();

  // stm32_gpiowrite(GPIO_MUX_EN, true);
  // stm32_gpiowrite(GPIO_SFM_CS, true);
  // stm32_gpiowrite(GPIO_SFM_MODE, true);


#endif /* CONFIG_STM32_SPI3 */

#ifdef CONFIG_STM32_SPI5
  spi5 = stm32_spibus_initialize(5);
  if (!spi5)
  {
    syslog(LOG_ERR, "[BRING_UP] ERROR: Failed to Initialize SPI 5 bus.\n");
  }
  else
  {
    syslog(LOG_INFO, "[BRING_UP] Initialized bus on SPI port 5.\n");
    SPI_SETFREQUENCY(spi5, 1000000);
    SPI_SETBITS(spi5, 8);
    SPI_SETMODE(spi5, SPIDEV_MODE0);

#ifdef CONFIG_SENSORS_MPU60X0
    struct mpu_config_s *mpu_config = NULL;
    //  SPI_SETFREQUENCY(spi5, 1000000);
    //   SPI_SETBITS(spi5, 8);
    //   SPI_SETMODE(spi5, SPIDEV_MODE0);
    printf("got here in config_sensoors_mpu6500");
    mpu_config = kmm_zalloc(sizeof(struct mpu_config_s));
    printf("the size of mpu_config is %d", sizeof(mpu_config));
    if (mpu_config == NULL)
    {
      printf("ERROR: Failed to allocate mpu60x0 driver\n");
    }
    else
    {
      printf("INside else\n");
      mpu_config->spi = spi5;
      mpu_config->spi_devid = SPIDEV_IMU(0);
      ret = mpu60x0_register("/dev/mpu6500", mpu_config);
      printf("the value of ret is : %d\n", ret);
      if (ret < 0)
      {
        printf("[bring up] failed to initialize driver of mppu 6500");
      }
      else
      {
        printf("[bringup ] successfully initialized driver of mpu 6500");
      }
    }
#endif // mpu configuration
    // SPI_SETFREQUENCY(spi5, 1000000);
    // SPI_SETBITS(spi5, 8);
    // SPI_SETMODE(spi5, SPIDEV_MODE0);
  }
#ifdef CONFIG_SENSORS_LIS3MDL
#ifdef CONFIG_UORB
  ret = lis3mdl_register(0, spi5, &mag0.dev); // since we're using uORB
#else
  ret = lis3mdl_register("dev/mag0", spi5, &mag0.dev);
#endif  //CONFIG_UORB
  if (ret < 0)
  {
    syslog(LOG_INFO, "[BRING_UP] Error: Failed to register LIS3MDL driver.\n");
  }
  else
  {
    syslog(LOG_INFO, "[BRING_UP] LIS3MDL registered on SPI 5.\n");
  }
#endif // CONFIG_SENSORS_LIS3MDL
#endif // CONFIG_STM32_SPI5

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC device. */

  ret = stm32_adc_setup();
  if (ret < 0)
  {
    syslog(LOG_ERR, "ERROR: stm32_adc_setup() failed: %d\n", ret);
  }
#endif

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

#if defined(CONFIG_MTD) && defined(CONFIG_MTD_PROGMEM)
  mtd = progmem_initialize();
  if (mtd == NULL)
  {
    syslog(LOG_ERR, "ERROR: progmem_initialize\n");
    printf("[BRINGUP: PROGMEM] error initializing progmem\n");
  }
  else
  {
    syslog(LOG_INFO, "INFO: Initialized progmem successfully: \n");
    printf("[BRINGUP: PROGMEM] Initialized progmem sucessfully...\r\n");
  }

  ret = register_mtddriver("/dev/intflash", mtd, 0, mtd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "ERROR: register_mtddriver() failed: %d\n", ret);
    printf("[BRINGUP: PROGMEM] Error registering mtd driver");
  }
  else
  {
    syslog(LOG_INFO, "INFO: registered mtd driver successfully \n");
    printf("[BRINGUP: PROGMEM] Registerd internal flash mtd driver successfullyy.....\r\n");
  }
#endif

#ifdef CONFIG_STM32_TIM6

  ret = stm32_timer_initialize("/dev/timer6", 6);
  if (ret < 0)
  {
    printf("failed to initialize /dev/timer6 : %d\n", ret);
  }
  else
  {
    printf("Timer 6 has been initialized successfully\n");
  }
#endif

#ifdef CONFIG_STM32_TIM7
  ret = stm32_timer_initialize("/dev/timer7", 7);
  if (ret < 0)
  {
    printf("failed to initialize /dev/timer7 : %d\n", ret);
  }
  else
  {
    printf("Timer 77 has been initialized successfully\n");
  }

#endif

#ifdef CONFIG_STM32_TIM8
  ret = stm32_timer_initialize("/dev/timer8", 8);
  if (ret < 0)
  {
    printf("failed to initialize /dev/timer8 : %d\n", ret);
  }
  else
  {
    printf("Timer 87 has been initialized successfully\n");
  }

#endif

#ifdef CONFIG_STM32_TIM9

  ret = stm32_timer_initialize("/dev/timer9", 9);
  if (ret < 0)
  {
    printf("failed to initialize /dev/timer9 : %d\n", ret);
  }
  else
  {
    printf("Timer 9 has been initialized successfully\n");
  }

#endif

#ifdef CONFIG_STM32_TIM10
  ret = stm32_timer_initialize("/dev/timer10", 10);
  if (ret < 0)
  {
    printf("failed to initialize /dev/timer10 : %d\n", ret);
  }
  else
  {
    printf("Timer 10 has been initialized successfully\n");
  }

#endif

#ifdef CONFIG_STM32_TIM11
  ret = stm32_timer_initialize("/dev/timer11", 11);
  if (ret < 0)
  {
    printf("failed to initialize /dev/timer11 : %d\n", ret);
  }
  else
  {
    printf("Timer 11 has been initialized successfully\n");
  }

#endif

#ifdef CONFIG_STM32_TIM12
  ret = stm32_timer_initialize("/dev/timer12", 12);
  if (ret < 0)
  {
    printf("failed to initialize /dev/timer12 : %d\n", ret);
  }
  else
  {
    printf("Timer 12 has been initialized successfully\n");
  }

#endif

#ifdef CONFIG_STM32_TIM13
  ret = stm32_timer_initialize("/dev/timer13", 13);
  if (ret < 0)
  {
    printf("failed to initialize /dev/timer13 : %d\n", ret);
  }
  else
  {
    printf("Timer 13 has been initialized successfully\n");
  }

#endif

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
