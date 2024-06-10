/****************************************************************************
 * boards/arm/stm32/stm32f427a-minimal/src/stm32_adc.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/analog/adc.h>

#include "stm32.h"
#include "stm32f427a.h"

#if defined(CONFIG_ADC) && (defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC3))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* 1 or 2 ADC devices (DEV1, DEV2).
 * ADC1 and ADC3 supported for now.
 */

#if defined(CONFIG_STM32_ADC1)
#  define DEV1_PORT 1
#endif

#if defined(CONFIG_STM32_ADC3)
#  if defined(DEV1_PORT)
#    define DEV2_PORT 3
#  else
#    define DEV1_PORT 3
#  endif
#endif

/* The number of ADC channels in the conversion list */

/* TODO DMA */

#define ADC1_NCHANNELS 16
#define ADC3_NCHANNELS 1

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DEV 1 */

#if DEV1_PORT == 1

#define DEV1_NCHANNELS ADC1_NCHANNELS

/* Identifying number of each ADC channel (even if NCHANNELS is less ) */

static const uint8_t g_chanlist1[ADC1_NCHANNELS] =
{
  0,  2,  3,  4,  5,  6,  7,  8,  9,  10,  11,  12,  14, 15, 17, 18
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist1[ADC1_NCHANNELS]  =
{
  GPIO_ADC1_IN0_0,                /* PA0      UNREG Current               ADC1/2/3 COMP. */
  GPIO_ADC1_IN2_0,                /* PA2      MAIN 3V3 Current            ADC1/2/3 COMP. */
  GPIO_ADC1_IN3_0,                /* PA3      3V3 COM Current             ADC1/2/3 COMP. */
  GPIO_ADC1_IN4_0,                /* PA4/N2   5V CURRENT                  ADC1/2 COMP. */
  GPIO_ADC1_IN5_0,                /* PA5/M3   BATTERY MONITOR             ADC1/2 COMP. */
  GPIO_ADC1_IN6_0,                /* PA6/N3   SOLAR PANEL 1 CURRENT       ADC1/2 COMP. */
  GPIO_ADC1_IN7_0,                /* PA7/K4   3V3 2 CURRENT               ADC1/2 COMP. */
  GPIO_ADC1_IN8_0,                /* PB0/N4   SOLAR PANEL 4 CURRENT       ADC1/2 COMP. */
  GPIO_ADC1_IN9_0,                /* PB1/K5   SOLAR PANEL 5 CURRENT       ADC1/2 COMP. */
  GPIO_ADC1_IN10_0,               /* PC0      BATTERY Current             ADC1/2/3 COMP. */
  GPIO_ADC1_IN11_0,               /* PC1      SOLAR Total Current         ADC1/2/3 COMP. */
  GPIO_ADC1_IN12_0,               /* PC2      RAW Current                 ADC1/2/3 COMP. */
  GPIO_ADC1_IN14_0,               /* PC4/L4   SOLAR PANEL 2 CURRENT       ADC1/2 COMP. */
  GPIO_ADC1_IN15_0                /* PC5/M4   SOLAR PANEL 3 CURRENT       ADC1/2 COMP. */
  // ADC1_IN17,                    /* VREF INT Internal voltage reference channel    ADC1   */
  // ADC1_IN18,                    /* TEMP_SENSE Internal Temperature sensor chanel  ADC1   */
};
#endif /* DEV1_PORT == 1 */

#ifdef DEV2_PORT

/* DEV 2 */

#if DEV2_PORT == 3

#define DEV2_NCHANNELS ADC3_NCHANNELS

/* Identifying number of each ADC channel */

static const uint8_t g_chanlist2[ADC3_NCHANNELS] =
{
  14,
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist2[ADC3_NCHANNELS] =
{
  GPIO_ADC3_IN14_0,         /* PF4  4V Current    ADC3 ONLY*/
};

#endif /* DEV2_PORT == 3 */
#endif /* DEV2_PORT */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int stm32_adc_setup(void)
{
  static bool initialized = false;
  struct adc_dev_s *adc1, *adc3;
  int ret;
  int i;

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* DEV1 */

      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < ADC1_NCHANNELS-2; i++)
        {
          stm32_configgpio(g_pinlist1[i]);
        }

      /* Call stm32_adcinitialize() to get an instance of the ADC interface */

      adc1 = stm32_adcinitialize(DEV1_PORT, g_chanlist1, 16);
      if (adc1 == NULL)
        {
          aerr("ERROR: Failed to get ADC interface 1\n");
          return -ENODEV;
        }

      /* Register the ADC driver at ADC1_PATH */

      ret = adc_register(ADC1_PATH, adc1);
      if (ret < 0)
        {
          aerr("ERROR: adc_register %s failed: %d\n", ADC1_PATH, ret);
          return ret;
        }

#ifdef DEV2_PORT
      /* DEV2 */

      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < ADC3_NCHANNELS; i++)
        {
          stm32_configgpio(g_pinlist2[i]);
        }

      /* Call stm32_adcinitialize() to get an instance of the ADC interface */

      adc3 = stm32_adcinitialize(DEV2_PORT, g_chanlist2, 1);
      if (adc3 == NULL)
        {
          aerr("ERROR: Failed to get ADC interface 2\n");
          return -ENODEV;
        }

      /* Register the ADC driver at ADC3_PATH */

      ret = adc_register(ADC3_PATH, adc3);
      if (ret < 0)
        {
          aerr("ERROR: adc_register /dev/adc1 failed: %d\n", ret);
          return ret;
        }
#endif

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_ADC && (CONFIG_STM32_ADC1 || CONFIG_STM32_ADC3) */
