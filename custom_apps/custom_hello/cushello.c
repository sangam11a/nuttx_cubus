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
#include "cushello.h"


static bool g_cushello_daemon_started;
/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_UORB

static void print_orb_mag_scaled_msg(FAR const struct orb_metadata *meta,
                                     FAR const void *buffer)
{
  FAR const struct orb_mag_scaled_s *message = buffer;
  const orb_abstime now = orb_absolute_time();

  uorbinfo_raw("%s:\ttimestamp: %"PRIu64" (%"PRIu64" us ago) X: %.4f Y: %.4f Z:%.4f Temp: %.4f",
                meta->o_name, message->timestamp, now - message->timestamp, 
                message->x, message->y, message->z, message->temperature);
}
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DEFINE(orb_mag_scaled, struct orb_mag_scaled_s, print_orb_mag_scaled_msg);

/****************************************************************************
 * custom_hello_main
 ****************************************************************************/

int  cushello_daemon(int argc, FAR char *argv[])
{
  
  g_cushello_daemon_started = true;
  syslog(LOG_INFO, "Hello World application for writing data to flash.\n");

  syslog(LOG_INFO, "opening a uORB topic to subscribe messages.\n");

  struct orb_mag_scaled_s mag_scaled;
  int instance =0;
  bool updated;
  int afd;

  struct sensor_mag mag0;
  struct pollfd fds;
  int fd;
  int ret;
  int i;

  /* advertie scaled mag topic */

  afd = orb_advertise_multi_queue_persist(ORB_ID(orb_mag_scaled),&mag_scaled,
                                           &instance, sizeof(struct orb_mag_scaled_s));
  if (afd < 0)
  {
    syslog(LOG_ERR, "Orb advertise failed.\n");
  }
  

  fd = orb_subscribe_multi(ORB_ID(sensor_mag), 0);

  fds.fd = fd;
  fds.events = POLLIN;
  for(;;)
  {
    if (poll(&fds, 1, 3000) > 0)
    {
      if(fds.revents & POLLIN)
      {
        ret = orb_copy_multi(fd, &mag0, sizeof(struct sensor_mag));
        if (ret < 0)
        {
          syslog(LOG_ERR, "ORB copy error, %d \n", ret);
          return ret;
        }

        // syslog(LOG_INFO, "Copied data from orb_object.\n");

        // printf("Timestamp: %lli \t", mag0.timestamp);
        // printf("Temperature: %0.02f \t", mag0.temperature);
        // printf("X : %0.02f \t", mag0.x);
        // printf("Y : %0.02f \t", mag0.y);
        // printf("Z : %0.02f \t\n", mag0.z);
      }
      mag_scaled.x = mag0.x * 100;
      mag_scaled.y = mag0.y * 100;
      mag_scaled.z = mag0.z * 100;
      mag_scaled.temperature = mag0.temperature - 50;
      mag_scaled.timestamp = orb_absolute_time();

      if(OK != orb_publish(ORB_ID(orb_mag_scaled), afd, &mag_scaled))
      {
        syslog(LOG_ERR,"Orb Publish failed\n");
      }
    }
  }
  
  ret = orb_unadvertise(afd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Orb Unadvertise failed.\n");
  }

  ret = orb_unsubscribe(fd);

  if (ret < 0)
  {
    syslog(LOG_ERR, "Orb unsubscribe Failed.\n");
  }
  
  return 0;
}



/****************************************************************************
 * custom_hello_thread
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
  int ret;

  printf("[Cushello] Starting task.\n");
  if (g_cushello_daemon_started)
  {
    printf("[Cushello] Task already started.\n");
  }

  ret = task_create("cushello_daemon",SCHED_PRIORITY_DEFAULT,
                    CONFIG_CUSTOM_APPS_CUSTOM_HELLO_STACKSIZE, cushello_daemon,
                    NULL);

  if (ret < 0)
  {
    int errcode = errno;
    printf("[cushello] ERROR: Failed to start cushello_dameon: %d\n",
           errcode);
    return EXIT_FAILURE;
  }

  printf("[cushello] cushello_daemon started\n");
  return EXIT_SUCCESS;
  
  
}