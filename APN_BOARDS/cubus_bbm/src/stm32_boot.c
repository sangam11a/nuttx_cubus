/****************************************************************************
 * boards/arm/stm32/stm32f42a-minimal/src/stm32_boot.c
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

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include <nuttx/spi/spi.h>
#include <stm32_spi.h>

#include <stdio.h>
#include <string.h>
#include <syslog.h>

#include "arm_internal.h"
#include "stm32f427a.h"
#include "stm32_ccm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
 void board_peripheral_reset(int ms)
{
	// Setting the kill switch control gpio

	stm32_gpiowrite(GPIO_KILL_SW_EN, 0);
	stm32_gpiowrite(GPIO_KILL_SW1_NEG, 0);
	stm32_gpiowrite(GPIO_KILL_SW1_POS, 0);
	stm32_gpiowrite(GPIO_KILL_SW2_NEG, 0);
	stm32_gpiowrite(GPIO_KILL_SW2_POS, 0);

	// setting watchdog and reset GPIOs
	stm32_gpiowrite(GPIO_GBL_RST, 0);
	// stm32_gpiowrite(GPIO_WD_WDI, 0); // setting initial watchdog pin status

	// setting power contorl pins
	stm32_gpiowrite(GPIO_DCDC_MSN_3V3_2_EN, 0); 	// enable MSN regulator
	stm32_gpiowrite(GPIO_DCDC_4V_EN, 0);		// Disable RF Power Amp regualtor
	stm32_gpiowrite(GPIO_DCDC_5V_EN, 0);		// Enable 5V regulator

	stm32_gpiowrite(GPIO_COM_4V_EN, 0);	// Disable RF Power Amp
	stm32_gpiowrite(GPIO_3V3_COM_EN, 0); 	// Enable COM systems
	stm32_gpiowrite(GPIO_MSN_3V3_EN, 0);	// Enable MSN Power
	stm32_gpiowrite(GPIO_MSN_5V_EN, 0);	// enable 5V Power Rail
	stm32_gpiowrite(GPIO_BURNER_EN, 0);	// Disable Burner enable
	stm32_gpiowrite(GPIO_UNREG_EN, 0);	// Disable UNREG power line

	// setting MSN control Pin
	stm32_gpiowrite(GPIO_MSN1_EN, 0);	//Disable MSN 1 activation
	stm32_gpiowrite(GPIO_MSN2_EN, 0);	//Disable MSN 2 activation
	stm32_gpiowrite(GPIO_MSN3_EN, 0);	//Disable MSN 3 activation

	// setting flash control pins
	stm32_gpiowrite(GPIO_SFM_MODE, 0);
	stm32_gpiowrite(GPIO_MUX_EN, 0);


	usleep(ms * 1000);
	syslog(LOG_DEBUG, "reset done, %d ms\n", ms);
  printf("Reset Done.\n");
}

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32_boardinitialize(void)
{
   	// Configure Watchdog and Reset control pins.
	// stm32_configgpio(GPIO_WD_WDI);
	stm32_configgpio(GPIO_GBL_RST);

	// Configure KILL Switch Control and Monitoring GPIOs
	stm32_configgpio(GPIO_KILL_SW_EN);
	stm32_configgpio(GPIO_KILL_SW1_STAT); //input
	stm32_configgpio(GPIO_KILL_SW1_NEG);
	stm32_configgpio(GPIO_KILL_SW1_POS);
	stm32_configgpio(GPIO_KILL_SW2_STAT); //input
	stm32_configgpio(GPIO_KILL_SW2_NEG);
	stm32_configgpio(GPIO_KILL_SW2_POS);

	// Configure power supply control pins
	stm32_configgpio(GPIO_DCDC_MSN_3V3_2_EN);
	stm32_configgpio(GPIO_DCDC_4V_EN);
	stm32_configgpio(GPIO_DCDC_5V_EN);

	stm32_configgpio(GPIO_3V3_COM_EN);
	stm32_configgpio(GPIO_MSN_3V3_EN);
	stm32_configgpio(GPIO_COM_4V_EN);
	stm32_configgpio(GPIO_MSN_5V_EN);
	stm32_configgpio(GPIO_BURNER_EN);
	stm32_configgpio(GPIO_UNREG_EN);

	// Configure MSN Control Pins
	stm32_configgpio(GPIO_MSN1_EN);
	stm32_configgpio(GPIO_MSN2_EN);
	stm32_configgpio(GPIO_MSN3_EN);

	// Configure Flash control pins
	stm32_configgpio(GPIO_SFM_MODE);
	stm32_configgpio(GPIO_MUX_EN);

	//configure MAG DRDY and MPU INT
	stm32_configgpio(GPIO_LIS3MDL_DRDY);
  stm32_configgpio(GPIO_LIS3MDL_CS);

  stm32_configgpio(GPIO_MPU_CS);
  stm32_configgpio(GPIO_LIS3MDL_INT);
  stm32_configgpio(GPIO_EXT_ADC1_CS);
	stm32_configgpio(GPIO_MPU_INT);
  
  stm32_configgpio(GPIO_MFM_CS);
  stm32_configgpio(GPIO_SFM_CS);
  stm32_configgpio(GPIO_MUX_EN);
  stm32_configgpio(GPIO_SFM_MODE);


#ifdef HAVE_CCM_HEAP
  /* Initialize CCM allocator */

  ccm_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board-specific initialization */

  stm32_bringup();
}
#endif


/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

static struct spi_dev_s *spi2;
static struct spi_dev_s *spi3;
static struct spi_dev_s *spi4;
static struct spi_dev_s *spi5;

int board_app_initialize(uintptr_t arg)
{

  printf("Initializing board applications.\n");
  board_peripheral_reset(10);

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || \
    defined(CONFIG_STM32_SPI3) || defined(CONFIG_STM32_SPI4) || \
    defined(CONFIG_STM32_SPI5)

  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function stm32_spidev_initialize() has been brought into the link.
   */

  if (stm32_spidev_initialize)
    {
      stm32_spidev_initialize(); 
    }
#endif

#ifdef CONFIG_BOARD_LATE_INITIALIZE
  /* Board initialization already performed by board_late_initialize() */

  return OK;
#else
  /* Perform board-specific initialization */

  return stm32_bringup();
  
#endif
}

