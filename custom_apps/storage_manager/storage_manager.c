
/****************************************************************************
 * custom_apps/storage_manager/storage_manager_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership. The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>
#include <uORB/uORB.h>
#include <sensor/mag.h>
#include <nuttx/sensors/sensor.h>
#include <poll.h>

#include "common_functions_edited.h"
#include "file_operations.h"

#define NB_LOWERHALFS 1
// #include <sensor/adc.h>
static struct work_s work_storage;
static int8_t count = 0;
static bool g_storage_manager_started;
int open_file_flash(struct file *file_pointer, char *flash_strpath, char *filename, int open_mode);
void read_and_print_mag_data(void)
{

  // adc
  //  struct ads7953_raw_msg e_ads7953_0;
  //  struct sat_temp_msg sat_temps;
  //  struct sat_volts_msg sat_volts;
  //  int ads,sat_temp,sat_volt;
  //
  if (work_queue(HPWORK, &work_storage, read_and_print_mag_data, NULL, SEC2TICK(30)) >= 0)
  {
    syslog(LOG_DEBUG, "Workqueue running\n");
  }
  else
  {
    if (work_queue(HPWORK, &work_storage, read_and_print_mag_data, NULL, SEC2TICK(30)) >= 0)
    {
      syslog(LOG_DEBUG, "Workqueue running\n");
    }
    else
    {
      syslog(LOG_DEBUG, "workqueue failled\n");
    }
  }
  // if (count == 0)
  // {
  //   Setup();
  // }
  count++;
  int sub_fd;
  struct orb_mag_scaled_s mag_data;
  satellite_health_s satellite_health;
  bool updated;

  sub_fd = orb_subscribe(ORB_ID(orb_mag_scaled));
  // ads = orb_subscibe(ORB_ID(e_ads7953_0));
  if (sub_fd < 0)
  {
    syslog(LOG_ERR, "Failed to subscribe to orb_mag_scaled topic\n");
    return;
  }

  // while (1)
  {
    orb_check(sub_fd, &updated);
    if (updated)
    {
      orb_copy(ORB_ID(orb_mag_scaled), sub_fd, &mag_data);

      syslog(LOG_INFO, "\n\n\r****-------[Storage maanager ]Mag Data:\n");

      syslog(LOG_INFO, "  ACCELEROMETER X: %.4f Y: %.4f Z: %.4f\n", mag_data.acc_x, mag_data.acc_y, mag_data.acc_z);
      syslog(LOG_INFO, "  GYROSCOPE X: %.4f Y: %.4f Z: %.4f\n", mag_data.gyro_x, mag_data.gyro_y, mag_data.gyro_z);

      syslog(LOG_INFO, "  MAGNETOMETER X: %.4f Y: %.4f Z: %.4f\n", mag_data.mag_x, mag_data.mag_y, mag_data.mag_z);
      syslog(LOG_INFO, "  Temp:  %.4f\n", mag_data.temperature);

      satellite_health.accl_x = mag_data.acc_x;
      satellite_health.accl_y = mag_data.acc_y;
      satellite_health.accl_z = mag_data.acc_z;

      satellite_health.gyro_x = mag_data.gyro_x;
      satellite_health.gyro_y = mag_data.gyro_y;
      satellite_health.gyro_z = mag_data.gyro_z;

      satellite_health.mag_x = mag_data.mag_x;
      satellite_health.mag_y = mag_data.mag_y;
      satellite_health.mag_z = mag_data.mag_z;

      store_sat_health_data(&satellite_health);
    }
    // sleep(20); // Sleep for 500 ms
  }

  orb_unsubscribe(sub_fd);
}

int storage_manager_daemon(int argc, FAR char *argv[])
{
  g_storage_manager_started = true;
  read_and_print_mag_data();
  return 0;
}

int main(int argc, FAR char *argv[])
{
  int ret;

  printf("[storage_manager] Starting task.\n");
  if (g_storage_manager_started)
  {
    printf("[storage_manager] Task already started.\n");
    return EXIT_SUCCESS;
  }

  // ret = task_create("storage_manager_daemon", SCHED_PRIORITY_DEFAULT, CONFIG_CUSTOM_APPS_STORAGE_MANAGER_STACKSIZE, storage_manager_daemon, NULL);
  ret = task_create("storage_manager_daemon", SCHED_PRIORITY_DEFAULT, CONFIG_CUSTOM_APPS_STORAGE_MANAGER_STACKSIZE, storage_manager_daemon, NULL);

  if (ret < 0)
  {
    int errcode = errno;
    printf("[storage_manager] ERROR: Failed to start storage_manager_daemon: %d\n", errcode);
    return EXIT_FAILURE;
  }

  printf("[storage_manager] storage_manager_daemon started\n");
  return EXIT_SUCCESS;
}

void store_sat_health_data(satellite_health_s *sat_health_data)
{
  struct file file_p;
  // TODO: discuss and figure out if we need to set limit to size of file and truncate contents once the file size limit is reached ...
  if (open_file_flash(&file_p, MFM_MAIN_STRPATH, file_name_sat_health, O_RDWR | O_APPEND) >= 0)
  {
    ssize_t bytes_written = file_write(&file_p, sat_health_data, sizeof(satellite_health_s));
    if (bytes_written > 0)
    {
      syslog(LOG_INFO, "Satellite Health data write Successful.\nData Len: %d.\n", bytes_written);
      file_close(&file_p);
    }
    else
    {
      syslog(LOG_INFO, "Write Failure.\n");
    }
    if (file_syncfs(&file_p) < 0)
    {
      syslog(LOG_DEBUG, "some issue while synfs closing\n");
      file_syncfs(&file_p);
    }
    if (file_close(&file_p) < 0)
    {
      syslog(LOG_DEBUG, "some issue while synfs closing\n");

      file_close(&file_p);
    }
  }
  else
  {
    syslog(LOG_ERR, "Error opening file to write satellite health data..\n");
  }
  file_close(&file_p);
}

int open_file_flash(struct file *file_pointer, char *flash_strpath, char *filename, int open_mode)
{

  const char file_name[] = {'\0'};
  // memcpy(file_name, filename, sizeof(filename));
  char path[65];
  sprintf(path, "%s%s", flash_strpath, filename);
  int fd = file_open(file_pointer, path, open_mode);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Error opening file: %s\n", path);
    return fd;
  }
  else
  {
    syslog(LOG_INFO, "Opened file: %s ...\n", path);
  }
  return fd;
}

//TODO: to make changes toread mag value
// void read_magnetometer(satellite_health_s *sat_health)
// {

//   int mag_fd;
//   uint16_t seconds;
//   int ret;

//   struct sensor_mag mag;
//   struct sensor_mag mag_sub;
//   int mag_afd, mag_sfd;

//   printf("SPI device LIS3MDL uorb test, World!\n");

//   /* Open SPI device driver */
//   mag_fd = open("/dev/uorb/sensor_mag0", O_RDONLY | O_NONBLOCK);
//   if (mag_fd < 0)
//   {
//     printf("Failed to open mag sensor\n");
//     return -1;
//   }

//   struct pollfd pfds[] = {
//       {.fd = mag_fd, .events = POLLIN}};

//   struct data sensor_data[] = {{.data_struct = &mag, .data_size = sizeof(struct sensor_mag)}};

//   {
//     ret = poll(pfds, NB_LOWERHALFS, -1);
//     if (ret < 0)
//     {
//       perror("Could not poll sensor\n");
//       return ret;
//     }

//     for (int i = 0; i < NB_LOWERHALFS; i++)
//     {
//       if (pfds[i].revents & POLLIN)
//       {
//         ret = read(pfds[i].fd, sensor_data[i].data_struct,
//                    sensor_data[i].data_size);

//         if (ret != sensor_data[i].data_size)
//         {
//           perror("Could not read from sub-sensor.");
//           return ret;
//         }
//       }
//     }
//     seconds -= 3;
//   }

//   printf("Timestamp = %lli\n"
//          "Temperature [c] = %f\n"
//          "mag x axis = %f\n"
//          "mag y axis = %f\n"
//          "mag z axis = %f\n",
//          mag.timestamp, mag.temperature, mag.mag_x, mag.mag_y, mag.mag_z);
//   sat_health->temp_obc = mag.temperature;
//   sat_health->mag_x = mag.mag_x;
//   sat_health->mag_y = mag.mag_y;
//   sat_health->mag_z = mag.mag_z;
//   close(mag_fd);
// }

//TODO WRITE A TASK OR WORK QUEUE FOR READING THE DATA/COMMAND EVERY SECOND AND PERFORM FLASH OPERATIONS

void flash_operations(){}