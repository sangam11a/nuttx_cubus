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

#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/lis3mdl.h>

#include "spi_driver_test.h"

#define NB_LOWERHALFS 1



struct data
{
  void *data_struct;
  uint16_t data_size;
};


// #define IOCTL_MODE 1
// #define READ_MODE   1

// #define MAX_CHANNELS 12

/****************************************************************************
 * spi_driver_test_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int mag_fd;
  uint16_t seconds;
  int ret;

  struct sensor_mag mag;
  struct sensor_mag mag_sub;

  int mag_afd, mag_sfd;

  printf("SPI device LIS3MDL uorb test, World!\n");

  /* Open SPI device driver */
  mag_fd = open("/dev/uorb/sensor_mag0", O_RDONLY | O_NONBLOCK);
  if (mag_fd < 0)
  {
    printf("Failed to open mag sensor\n");
    return -1;
  }

  struct pollfd pfds[] = {
      {.fd = mag_fd, .events = POLLIN}};

  struct data sensor_data[] = {
      {.data_struct = &mag, .data_size = sizeof(struct sensor_mag)}};

  seconds = 5*3;

  while (seconds > 0)
  {
    ret = poll(pfds, NB_LOWERHALFS, -1);
    if (ret < 0)
    {
      perror("Could not poll sensor\n");
      return ret;
    }

    for (int i = 0; i < NB_LOWERHALFS; i++)
    {
      if (pfds[i].revents & POLLIN)
      {
        ret = read(pfds[i].fd, sensor_data[i].data_struct,
                   sensor_data[i].data_size);

        if (ret != sensor_data[i].data_size)
        {
          perror("Could not read from sub-sensor.");
          return ret;
        }
      }
    }
    seconds -= 3;
  }

  printf("Timestamp = %lli\n"
         "Temperature [c] = %f\n"
         "mag x axis = %f\n"
         "mag y axis = %f\n"
         "mag z axis = %f\n",
         mag.timestamp, mag.temperature, mag.x, mag.y, mag.z);

  close(mag_fd);

  // mag_afd = orb_advertise(ORB_ID(sensor_mag),&mag);
  // if (mag_afd < 0)
  // {
  //   printf("advertise failed: %d\n", errno);
  // }

  // orb_publish(ORB_ID(sensor_mag), mag_afd, &mag);
  

  // mag_sfd = orb_subscribe(ORB_ID(sensor_mag));
  // if (mag_sfd < 0)
  // {
  //   printf("subscribe failed: %d\n", errno);
  // }
  
  // if (OK != orb_copy(ORB_ID(sensor_mag), mag_sfd, &mag_sub))
  // {
  //   printf("copy failed: %d\n", errno);
  // }
  
  // if(mag_sub.timestamp != mag.timestamp)
  // {
  //   printf("mismatch adv val: %lli subb val: %lli\n", mag.timestamp, mag_sub.timestamp);
  // } 

  return 0;
}
