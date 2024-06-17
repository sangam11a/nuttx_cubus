/****************************************************************************
 * boards/xtensa/stm32/common/src/stm32_etx_led.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/arch.h>
#include "stm32.h"
#include "stm32_own_led.h"

#ifdef CONFIG_STM32_OWN_LED

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int     etx_gpio_open(FAR struct file *filep);
static int     etx_gpio_close(FAR struct file *filep);
static ssize_t etx_gpio_read(FAR struct file *filep, FAR const char *buffer, size_t len);
static ssize_t etx_gpio_write(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen);


/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations etx_gpio_fops =
{
  etx_gpio_open,   /* open   */
  etx_gpio_close,  /* close  */
  etx_gpio_read,           /* read   */
  etx_gpio_write,  /* write  */
  NULL,           /* seek   */
  NULL,           /* ioctl  */
  NULL,           /* poll   */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL            /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: etx_led_open
 *
 * Description:
 *   This function will be getting called when the application
 *  opens the "/dev/etx_led" file.
 ****************************************************************************/

static int etx_gpio_open(FAR struct file *filep)
{
  int ret = 0;

  return ( ret );
}

/****************************************************************************
 * Name: etx_led_close
 *
 * Description:
 *   This function will be getting called when the application
 *  closess the "/dev/etx_led" file.
 ****************************************************************************/

static int etx_gpio_close(FAR struct file *filep)
{
  int ret = 0;

  return ( ret );
}

/****************************************************************************
 * Name: etx_led_write
 *
 * Description:
 *   This function will be getting called when the application
 *  writes data the "/dev/etx_led" file.
 ****************************************************************************/

static ssize_t etx_gpio_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  DEBUGASSERT(buffer != NULL);
  
  //buffer[0] contains the value (ON/OFF)
  //buffer[1] contains the GPIO Number

  gpio *etx_gpio = (FAR gpio *)((uintptr_t)buffer);
  int32_t value     = etx_gpio->gpio_val;
  int32_t number    = etx_gpio->gpio_num;
  syslog(LOG_INFO, "GPIO number: %d value %d \n", number, value);
  // Configure the GPIO 
  stm32_configgpio(number);
  // Write to the GPIO
  stm32_gpiowrite(number, value);

  return (len);
}

static ssize_t etx_gpio_read(FAR struct file *filep, FAR const char *buffer, size_t len){
  int ret = 0;
  DEBUGASSERT(buffer != NULL);

  gpio *etx_gpio_read = (FAR gpio*)((uintptr_t)buffer);
  etx_gpio_read->gpio_val = stm32_gpioread(etx_gpio_read->gpio_num);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: etx_led_driver_init
 *
 * Description:
 *   Initialize the EmbeTronicX LED device Driver. 
 *   This will create the device file as "/dev/etx_led"
 * 
 *   return negative number on failure.
 *   return 0 on success.
 *
 ****************************************************************************/

int etx_gpio_driver_init( void )
{
  int ret = 0;
  ret = register_driver("/dev/gpio_rw", &etx_gpio_fops, 0666, NULL);
  if (ret < 0)
  {
    _err("ERROR: register_driver failed : /dev/gpio_rw : %d\n", ret);
    ret = -1;;
  }

  return ( ret );

}

#endif /* CONFIG_stm32_ETX_LED */