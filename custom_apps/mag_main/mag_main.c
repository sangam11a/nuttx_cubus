/****************************************************************************
 * custom_apps/mag_main/custom_hello.c
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
#include "mag_main.h"
#include <fcntl.h>

#define REG_LOW_MASK 0xFF00
#define REG_HIGH_MASK 0x00FF
#define MPU6050_FS_SEL 32.8f
#define MPU6050_AFS_SEL 4096.0f

#ifndef CONFIG_MPU_PATH
#define CONFIG_MPU_PATH "/dev/mpu6500"
#endif
struct mpu6500_imu_msg
{
  uint64_t timestamp;
  float acc_x;
  float acc_y;
  float acc_z;
  float temp;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float mag_y;
  float mag_z;
  float temperature;
  int ANT_DEPLOYED;
};

void read_mpu6500_2(int fd, struct sensor_accel *acc_data, struct sensor_gyro *gyro_data, struct mpu6500_imu_msg *raw_imu)
{
  int16_t raw_data[7], conv_data[7];
  memset(raw_imu, 0, sizeof(struct mpu6500_imu_msg));
  int ret = read(fd, raw_data, sizeof(raw_data));
  if (ret <= 0)
  {
    printf("Failed to read accelerometer data\n");
  }
  else
  {
    conv_data[0] = ((raw_data[0] & REG_HIGH_MASK) << 8) + ((raw_data[0] & REG_LOW_MASK) >> 8);
    conv_data[1] = ((raw_data[1] & REG_HIGH_MASK) << 8) + ((raw_data[1] & REG_LOW_MASK) >> 8);
    conv_data[2] = ((raw_data[2] & REG_HIGH_MASK) << 8) + ((raw_data[2] & REG_LOW_MASK) >> 8);
    conv_data[3] = ((raw_data[4] & REG_HIGH_MASK) << 8) + ((raw_data[4] & REG_LOW_MASK) >> 8);
    conv_data[4] = ((raw_data[5] & REG_HIGH_MASK) << 8) + ((raw_data[5] & REG_LOW_MASK) >> 8);
    conv_data[5] = ((raw_data[6] & REG_HIGH_MASK) << 8) + ((raw_data[6] & REG_LOW_MASK) >> 8);
  }
  acc_data->x = conv_data[0] * 9.8 / MPU6050_AFS_SEL;
  acc_data->y = conv_data[1] * 9.8 / MPU6050_AFS_SEL;
  acc_data->z = conv_data[2] * 9.8 / MPU6050_AFS_SEL;

  gyro_data->x = conv_data[3] / MPU6050_FS_SEL;
  gyro_data->y = conv_data[4] / MPU6050_FS_SEL;
  gyro_data->z = conv_data[5] / MPU6050_FS_SEL;

  raw_imu->acc_x = acc_data->x;
  raw_imu->acc_y = acc_data->y;
  raw_imu->acc_z = acc_data->z;

  raw_imu->gyro_x = (float)gyro_data->x;
  raw_imu->gyro_y = (float)gyro_data->y;
  raw_imu->gyro_z = (float)gyro_data->z;
}

static bool g_mag_daemon_started;
/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_UORB

static void print_orb_mag_scaled_msg(FAR const struct orb_metadata *meta,
                                     FAR const void *buffer)
{
  FAR const struct orb_mag_scaled_s *message = buffer;
  const orb_abstime now = orb_absolute_time();

  // uorbinfo_raw("%s:\ttimestamp: %"PRIu64" (%"PRIu64" us ago) X: %.4f Y: %.4f Z:%.4f Temp: %.4f",
  //               meta->o_name, message->timestamp, now - message->timestamp,
  //               message->x, message->y, message->z, message->temperature);
}
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DEFINE(orb_mag_scaled, struct orb_mag_scaled_s, print_orb_mag_scaled_msg);

/****************************************************************************
 * custom_hello_main
 ****************************************************************************/

int mag_daemon(int argc, FAR char *argv[])
{

  g_mag_daemon_started = true;
  struct orb_mag_scaled_s mag_scaled;
  int instance = 0;
  bool updated;
  int afd;

  struct sensor_mag mag0;
  struct pollfd fds;
  int fd;
  int ret;
  int i;
  uint16_t mag[3];

  /* advertie scaled mag topic */

  afd = orb_advertise_multi_queue_persist(ORB_ID(orb_mag_scaled), &mag_scaled,
                                          &instance, sizeof(struct orb_mag_scaled_s));
  if (afd < 0)
  {
    syslog(LOG_ERR, "Orb advertise failed.\n");
  }

  fd = orb_subscribe_multi(ORB_ID(sensor_mag), 0);

  fds.fd = fd;
  fds.events = POLLIN;

  //
  struct sensor_accel imu_acc_data;
  struct sensor_gyro imu_gyro_data;
  struct mpu6500_imu_msg raw_imu;

  int fd1 = open("/dev/mpu6500", O_RDONLY);
  if (fd1 < 0)
  {
    printf("Failed to open mpu6500\n");
    return -1;
  }

  //

  // for (;;)
  // {
  //   read_mpu6500_2(fd1, &imu_acc_data, &imu_gyro_data, &raw_imu);

  //   // printf(
  //   //   // "Timestamp: %f  Temperature: %f\n"
  //   //        "Accelerometer X: %f | Y: %f | Z: %f\n"
  //   //        "Gyroscope X: %f | Y: %f | Z: %f\n",
  //   //       //  imu_acc_data.timestamp, imu_acc_data.temperature,
  //   //          raw_imu.acc_x,   raw_imu.acc_y,   raw_imu.acc_z,
  //   //        raw_imu.gyro_x, raw_imu.gyro_y, raw_imu.gyro_z);

  //   if (poll(&fds, 1, 3000) > 0)
  //   {
  //     if (fds.revents & POLLIN)
  //     {
  //       ret = orb_copy_multi(fd, &mag0, sizeof(struct sensor_mag));
  //       if (ret < 0)
  //       {
  //         syslog(LOG_ERR, "ORB copy error, %d \n", ret);
  //         return ret;
  //       }

  //       // syslog(LOG_INFO, "Copied data from orb_object.\n");

  //       // printf("Timestamp: %lli \t", mag0.timestamp);
  //       // printf("Temperature: %0.02f \t", mag0.temperature);
  //     }
  //     mag_scaled.mag_x = mag0.mag_x * 0.058f;
  //     mag_scaled.mag_y = mag0.mag_y * 0.058f;
  //     mag_scaled.mag_z = mag0.mag_z * 0.058f;
  //     // printf("Mag X : %0.02f \t", mag_scaled.mag_x);
  //     // printf("Y : %0.02f \t", mag_scaled.mag_y);
  //     // printf("Z : %0.02f \t\n", mag_scaled.mag_z);
  //     mag_scaled.temperature = mag0.temperature - 50;
  //     mag_scaled.timestamp = orb_absolute_time();

  //     raw_imu.mag_x = mag_scaled.mag_x;
  //     raw_imu.mag_y = mag_scaled.mag_y;
  //     raw_imu.mag_z = mag_scaled.mag_z;
  //     // mag[0] =(in)
  //     // Assign scaled magnetic field values to raw IMU data
  //     raw_imu.mag_x = mag_scaled.mag_x;
  //     raw_imu.mag_y = mag_scaled.mag_y;
  //     raw_imu.mag_z = mag_scaled.mag_z;

  //     // Convert scaled values to uint16_t and multiply by 100
  //     mag[0] = (uint16_t)(mag_scaled.mag_x * 100.0f); // For X-axis
  //     mag[1] = (uint16_t)(mag_scaled.mag_y * 100.0f); // For Y-axis
  //     mag[2] = (uint16_t)(mag_scaled.mag_z * 100.0f); // For Z-axis

  //     raw_imu.temperature = mag_scaled.temperature;
  //     raw_imu.timestamp = mag_scaled.timestamp;
  //     if (OK != orb_publish(ORB_ID(orb_mag_scaled), afd, &raw_imu))
  //     {
  //       syslog(LOG_ERR, "Orb Publish failed\n");
  //     }
  //   }
  // }
  for (;;)
{
    read_mpu6500_2(fd1, &imu_acc_data, &imu_gyro_data, &raw_imu);

    if (poll(&fds, 1, 10) > 0) // Poll every 10 ms
    {
        if (fds.revents & POLLIN)
        {
            ret = orb_copy_multi(fd, &mag0, sizeof(struct sensor_mag));
            if (ret < 0)
            {
                syslog(LOG_ERR, "ORB copy error, %d \n", ret);
                return ret;
            }

            mag_scaled.mag_x = mag0.mag_x * 0.058f;
            mag_scaled.mag_y = mag0.mag_y * 0.058f;
            mag_scaled.mag_z = mag0.mag_z * 0.058f;
            mag_scaled.temperature = mag0.temperature - 50;
            mag_scaled.timestamp = orb_absolute_time();

            raw_imu.mag_x = mag_scaled.mag_x;
            raw_imu.mag_y = mag_scaled.mag_y;
            raw_imu.mag_z = mag_scaled.mag_z;
            raw_imu.temperature = mag_scaled.temperature;
            raw_imu.timestamp = mag_scaled.timestamp;

            if (OK != orb_publish(ORB_ID(orb_mag_scaled), afd, &raw_imu))
            {
                syslog(LOG_ERR, "Orb Publish failed\n");
            }
        }
    }

    // Ensure loop runs every 10 ms
    usleep(10000); // 10 ms delay
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
 * mag_main thread
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
  int ret;

  printf("[mag] Starting task.\n");
  if (g_mag_daemon_started)
  {
    printf("[mag] Task already started.\n");
    return EXIT_SUCCESS;
  }
  toggle_wdg();
  ret = task_create("mag_daemon", 100,
                    CONFIG_CUSTOM_APPS_MAG_MAIN_STACKSIZE, mag_daemon,
                    NULL);

  if (ret < 0)
  {
    int errcode = errno;
    printf("[mag] ERROR: Failed to start mag_dameon: %d\n",
           errcode);
    return EXIT_FAILURE;
  }

  printf("[mag] mag_daemon started\n");
}