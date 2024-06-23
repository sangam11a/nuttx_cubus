/****************************************************************************
 * custom_apps/custom_hello/custom_hello.c
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
#include <stdio.h>
#include <assert.h>
#include <fcntl.h>
#include <time.h>
#include <poll.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "gpio_control.h"
#include "gpio_definitions.h"
/****************************************************************************
 * spi_driver_test_main
 ****************************************************************************/

int gpio_write(uint32_t pin, uint8_t mode)
{

  gpio_config_s gpio_numval;
  int fd = open(ETX_LED_DRIVER_PATH, O_WRONLY);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Error opening %s for GPIO WRITE...", ETX_LED_DRIVER_PATH);
    close(fd);
    return -1;
  }
  gpio_numval.gpio_num = pin;
  gpio_numval.gpio_val = mode;
  if (gpio_numval.gpio_val > 1 || gpio_numval.gpio_num < 0)
  {
    syslog(LOG_ERR, "Undefined GPIO pin or set mode selected...\n");
    return -2;
  }
  int ret = write(fd, (const void *)&gpio_numval, sizeof(gpio_config_s));
  close(fd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Unable to write to gpio pin...\n");
  }
  return ret;
}

int main(int argc, FAR char *argv[])
{
  if (argc < 3)
  {
    printf("Enter both subsystem name and gpio mode\n Enter: Application Name, Subsystem name (CAM, MSN1, MSN2, MSN3 etc.), GPIO Pin Mode (1 or 0)\n");
    return -1;
  }
  uint8_t pin_mode = atoi(argv[2]);
  if (!strcmp(argv[1], "COM"))
  {
    gpio_write(GPIO_3V3_COM_EN, pin_mode);
  }
  else if (!strcmp(argv[1], "MSN1"))
  { // ADCS
    gpio_write(GPIO_MSN_3V3_EN, pin_mode);
    gpio_write(GPIO_MSN1_EN, pin_mode);
  }
  else if (!strcmp(argv[1], "MSN2"))
  { // CAM
    gpio_write(GPIO_MSN_3V3_EN, pin_mode);
    gpio_write(GPIO_MSN2_EN, pin_mode);
  }
  else if (!strcmp(argv[1], "MSN3"))
  { // EPDM
    gpio_write(GPIO_MSN_3V3_EN, pin_mode);
    gpio_write(GPIO_MSN3_EN, pin_mode);
  }
  else
  { // keep on adding other gpio pins as you go
    printf("Unknown command \n");
    return -2;
  }
}
