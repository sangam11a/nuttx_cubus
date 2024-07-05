/****************************************************************************
 * custom_apps/ads7953/ads7953_main.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <nuttx/analog/ads7953.h>
#include <nuttx/analog/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include <time.h>
#include <fcntl.h>

#include <poll.h>
#include <sensor/adc.h>
#include <uORB/uORB.h>
#include <sched.h>
#include <math.h>


#define VOLT_DIV_RATIO ((1100 + 931) / 931) // ratio of voltage divider used

static bool g_ads7953_daemon_started;

/****************************************************************************
 * Public Data
 ****************************************************************************/

int ads7953_daemon(int argc, FAR char *argv[])
{
  g_ads7953_daemon_started = true;
  int ret;
  int adc_instance = 0;
  int fd;
  int raw_afd;
  int temp_afd;
  int volts_afd;
  uint8_t raw_temp[2];

  struct ads7953_raw_msg e_ads7953_0;
  struct sat_temp_msg sat_temps;
  struct sat_volts_msg sat_volts;

  raw_afd = orb_advertise_multi_queue_persist(ORB_ID(ads7953_raw_msg), &e_ads7953_0,
                                              &adc_instance, sizeof(struct ads7953_raw_msg));

  if (raw_afd < 0)
  {
    syslog(LOG_ERR, "Raw ADC advertise failed. %i\n", raw_afd);
  }
  temp_afd = orb_advertise_multi_queue_persist(ORB_ID(sat_temp_msg), &sat_temps, 0,
                                               sizeof(struct sat_temp_msg));
  if (temp_afd < 0)
  {
    syslog(LOG_ERR, "sat temps advertise failed. %i\n", temp_afd);
  }

  volts_afd = orb_advertise_multi_queue_persist(ORB_ID(sat_volts_msg), &sat_volts,
                                                0, sizeof(struct sat_volts_msg));
  if (volts_afd < 0)
  {
    syslog(LOG_ERR, "Sat Volts advertise failed. %i\n", volts_afd);
  }
  for (;;)
  {
    fd = open(CONFIG_EADC0_PATH, O_RDONLY);
    if (fd < 0)
    {
      syslog(LOG_ERR, "[ads79853] failed to open ads7953.\n");
      goto error;
    }

    ret = ioctl(fd, ANIOC_ADC_MANUAL_SELECT, NULL);
    usleep(10);
    ret = ioctl(fd, ANIOC_ADC_AUTO_2_SELECT, NULL);
    usleep(10);
    ret = ioctl(fd, ANIOC_ADC_AUTO_2_PROGRAM, NULL);
    usleep(1000);

#ifdef CONFIG_IOCTL_MODE
    for (int i = 0; i < CONFIG_EADC0_CHAN_NO; i++)
    {
      ioctl(fd, ANIOC_ADC_AUTO_2_SELECT_READ, raw_temp);
      /* Volts channel till no i =6, tempe channel from i = 8 to i = 15 */
      if (i < 7)
      {

        e_ads7953_0.volts_chan[i] = (raw_temp[0] << 8 | raw_temp[1]) & 0x0fff;
        e_ads7953_0.volts_chan_volts[i] = e_ads7953_0.volts_chan[i] * 2.5 / 4095;
      }
      else if (i > 7)
      {

        e_ads7953_0.temp_chan[i - 8] = (raw_temp[0] << 8 | raw_temp[1]) & 0x0fff;
        e_ads7953_0.temp_chan_volts[i - 8] = e_ads7953_0.temp_chan[i - 8] * 2.5 / 4095;
      }
    }
#else
    for (int i = 0; i < CONFIG_EADC0_CHAN_NO; i++)
    {
      int ret1 = read(fd, &raw_temp, 2);
      if (ret1 < 0)
      {
        syslog(LOG_ERR, "Error reading from E-ADC.\n");
        goto rd_err;
      }
      if (i < 7)
      {
        e_ads7953_0.volts_chan[i] = (raw_temp[0] << 8 | raw_temp[1]) & 0x0fff;
        e_ads7953_0.volts_chan_volts[i] = e_ads7953_0.volts_chan[i] * 2.5 / 4095;
      }
      else if (i > 7)
      {
        e_ads7953_0.temp_chan[i - 8] = (raw_temp[0] << 8 | raw_temp[1]) & 0x0fff;
        e_ads7953_0.temp_chan_volts[i - 8] = e_ads7953_0.temp_chan[i - 8] * 2.5 / 4095;
      }
    }
#endif

    close(fd); // closing ADC sesnor path after reading is completed

    e_ads7953_0.timestamp = orb_absolute_time();

    if (OK != orb_publish(ORB_ID(ads7953_raw_msg), raw_afd, &e_ads7953_0))
    {
      syslog(LOG_ERR, "Orb Publish failed\n");
    }

    // Processing temperature sensor reading
    float temp = e_ads7953_0.temp_chan_volts[1] * 10000 / (2.5 - e_ads7953_0.temp_chan_volts[1]);
    sat_temps.batt_temp = 3976 * 298 / (3976 - (298 * log(10000 / temp)));
    temp = sqrtf((5.506 * 5.506) + (4 * 0.00176 * (870.6 - (e_ads7953_0.temp_chan_volts[2] * 1000))));
    sat_temps.temp_bpb = (((5.506 - temp / (2 * (-0.00176))) + 30)) * 100;
    temp = sqrtf((5.506 * 5.506) + (4 * 0.00176 * (870.6 - (e_ads7953_0.temp_chan_volts[0] * 1000))));
    sat_temps.temp_ant = (((5.506 - temp / (2 * (-0.00176))) + 30)) * 100;
    temp = sqrtf((5.506 * 5.506) + (4 * 0.00176 * (870.6 - (e_ads7953_0.temp_chan_volts[3] * 1000))));
    sat_temps.temp_z_pos = (((5.506 - temp / (2 * (-0.00176))) + 30)) * 100;
    temp = sqrtf((5.506 * 5.506) + (4 * 0.00176 * (870.6 - (e_ads7953_0.temp_chan_volts[4] * 1000))));
    sat_temps.temp_5 = (((5.506 - temp / (2 * (-0.00176))) + 30)) * 100;
    temp = sqrtf((5.506 * 5.506) + (4 * 0.00176 * (870.6 - (e_ads7953_0.temp_chan_volts[5] * 1000))));
    sat_temps.temp_4 = (((5.506 - temp / (2 * (-0.00176))) + 30)) * 100;
    temp = sqrtf((5.506 * 5.506) + (4 * 0.00176 * (870.6 - (e_ads7953_0.temp_chan_volts[6] * 1000))));
    sat_temps.temp_3 = (((5.506 - temp / (2 * (-0.00176))) + 30)) * 100;
    temp = sqrtf((5.506 * 5.506) + (4 * 0.00176 * (870.6 - (e_ads7953_0.temp_chan_volts[7] * 1000))));
    sat_temps.temp_2 = (((5.506 - temp / (2 * (-0.00176))) + 30)) * 100;
    sat_temps.timestamp = e_ads7953_0.timestamp;

    // temperature sensor data processed
    if (OK != orb_publish(ORB_ID(sat_temp_msg), temp_afd, &sat_temps))
    {
      syslog(LOG_ERR, "Sat temp Orb Publish failed\n");
    }

    /* Processing Voltage sensor data */

    sat_volts.volt_SolT = e_ads7953_0.volts_chan_volts[0] * VOLT_DIV_RATIO;
    sat_volts.volt_raw = e_ads7953_0.volts_chan_volts[1] * VOLT_DIV_RATIO;
    sat_volts.volt_sp5 = e_ads7953_0.volts_chan_volts[2] * VOLT_DIV_RATIO;
    sat_volts.volt_sp4 = e_ads7953_0.volts_chan_volts[3] * VOLT_DIV_RATIO;
    sat_volts.volt_sp3 = e_ads7953_0.volts_chan_volts[4] * VOLT_DIV_RATIO;
    sat_volts.volt_sp1 = e_ads7953_0.volts_chan_volts[5] * VOLT_DIV_RATIO;
    sat_volts.volt_sp2 = e_ads7953_0.volts_chan_volts[6] * VOLT_DIV_RATIO;
    sat_volts.timestamp = e_ads7953_0.timestamp;

    /* Voltage sensor data processed */
    if (OK != orb_publish(ORB_ID(sat_volts_msg), volts_afd, &sat_volts))
    {
      syslog(LOG_ERR, "Sat volts Orb Publish failed\n");
    }

    usleep(500000);
  } // for(;;)

  ret = orb_unadvertise(raw_afd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "adc raw Orb Unadvertise failed.\n");
  }
  ret = orb_unadvertise(temp_afd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Sat temps Orb Unadvertise failed.\n");
  }
  ret = orb_unadvertise(volts_afd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Sat volts Orb Unadvertise failed.\n");
  }
rd_err:
  close(fd);
error:
  return 0;
}

int main(int argc, FAR char *argv[])
{
  int ret;

  syslog(LOG_INFO,"[ads7953] Starting task.\n");
  if (g_ads7953_daemon_started)
  {
    syslog(LOG_WARNING,"[ads7953] Task already started.\n");
    return EXIT_SUCCESS;
  }

  ret = task_create("ads7953_daemon", SCHED_PRIORITY_DEFAULT,
                    CONFIG_CUSTOM_APPS_ADS7953_STACKSIZE, ads7953_daemon,
                    NULL);

  if (ret < 0)
  {
    int errcode = errno;
    syslog(LOG_ERR,"[ads7953] ERROR: Failed to start ads7953_dameon: %d\n",
           errcode);
    return EXIT_FAILURE;
  }

  syslog(LOG_INFO,"[ads7953] ads7953_daemon started\n");
  return EXIT_SUCCESS;
}