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


#include <nuttx/sensors/lis3mdl.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/analog/ads7953.h>


// #define IOCTL_MODE  1
// #define READ_MODE   1

// #define MAX_CHANNELS  12
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * custom_hello_main
 ****************************************************************************/

int  main(int argc, FAR char *argv[])
{
  printf("SPI device LIS3MDL test, World!\n");

  /* Open SPI device driver */
  int16_t mag_data[4];
  int fd = open("/dev/mag0", O_RDWR);
  assert(fd >= 0);
  int data_size = read(fd, mag_data,8);
  if (data_size > 0)
  {
    printf("read sensor data from Mag. Len %i \n", data_size);
    printf("read data %d %d %d %d\n", mag_data[0], mag_data[1], mag_data[2],
             mag_data[3]);
  } else  {
    printf("Failed to read from sensor.\n");
  }

  close(fd);
  return 0;
}
