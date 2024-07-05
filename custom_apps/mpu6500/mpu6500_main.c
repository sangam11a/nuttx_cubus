/****************************************************************************
 * custom_apps/mpu6500/mpu6500_main.c
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
#include <sensor/accel.h>
#include <sensor/gyro.h>
#include <nuttx/sensors/sensor.h>
#include <uORB/uORB.h>
#include <sched.h>

#define REG_LOW_MASK 0xFF00
#define REG_HIGH_MASK 0x00FF
#define MPU6500_FS_SEL 32.8f
#define MPU6500_AFS_SEL 4096.0f
#define DEVNAME_SIZE 32

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
  float temperature;
};

ORB_DECLARE(mpu6500_imu_msg);

static bool g_mpu6500_daemon_started;

#ifdef CONFIG_DEBUG_UORB

static void print_mpu6500_imu_msg(FAR const struct orb_metadata *meta,
                                  FAR const void *buffer)
{
  FAR const struct mpu6500_imu_msg *message = buffer;
  const orb_abstime now = orb_absolute_time();

  uorbinfo_raw("%s:\ttimestamp: %" PRIu64 " (%" PRIu64 " us ago) acc_X: %.4f acc_Y: %.4f acc_Z:%.4f Temp: %.4f \n "
               "gyro_X: %0.4f gyro_Y: %0.4f gyro_Z: %0.4f temp: %0.2f\n",
               meta->o_name, message->timestamp, now - message->timestamp,
               message->acc_x, message->acc_y, message->acc_z, message->gyro_x, message->gyro_y, message->gyro_z, message->temperature);
}
#endif


/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DEFINE(mpu6500_imu_msg, struct mpu6500_imu_msg, print_mpu6500_imu_msg);

int mpu6500_daemon(int argc, FAR char *argv[])
{
  g_mpu6500_daemon_started = true;
  int fd;
  int afd;
  int ret;
  int instance = 0;

  int16_t imu_raw[7];
  struct mpu6500_imu_msg imu0;

  afd = orb_advertise_multi_queue_persist(ORB_ID(mpu6500_imu_msg), &imu0,
                                          &instance, sizeof(struct mpu6500_imu_msg));
  if (afd < 0)
  {
    syslog(LOG_ERR, "Orb advertise failed.\n");
  }
for (;;)
{
  fd = open(CONFIG_IMU0_PATH, O_RDONLY);
  if (fd < 0)
  {
    syslog(LOG_ERR, "[mpu6500] failed to open mpu6500.");
    goto error;
  }

  ret = read(fd, imu_raw, sizeof(imu_raw));

  if (ret != sizeof(imu_raw))
  {
    syslog(LOG_ERR, "Failed to read IMU data.\n");
    goto rd_err;
  }

  imu0.acc_x = (((imu_raw[0] & REG_HIGH_MASK) << 8) + ((imu_raw[0] & REG_LOW_MASK) >> 8)) / MPU6500_AFS_SEL;
  imu0.acc_y = (((imu_raw[1] & REG_HIGH_MASK) << 8) + ((imu_raw[1] & REG_LOW_MASK) >> 8)) / MPU6500_AFS_SEL;
  imu0.acc_z = (((imu_raw[2] & REG_HIGH_MASK) << 8) + ((imu_raw[2] & REG_LOW_MASK) >> 8)) / MPU6500_AFS_SEL;

  imu0.gyro_x = (((imu_raw[4] & REG_HIGH_MASK) << 8) + ((imu_raw[4] & REG_LOW_MASK) >> 8)) / MPU6500_FS_SEL;
  imu0.gyro_y = (((imu_raw[5] & REG_HIGH_MASK) << 8) + ((imu_raw[5] & REG_LOW_MASK) >> 8)) / MPU6500_FS_SEL;
  imu0.gyro_z = (((imu_raw[6] & REG_HIGH_MASK) << 8) + ((imu_raw[6] & REG_LOW_MASK) >> 8)) / MPU6500_FS_SEL;
  imu0.timestamp = orb_absolute_time();
   
  close(fd);

  if (OK != orb_publish(ORB_ID(mpu6500_imu_msg), afd, &imu0))
  {
    syslog(LOG_ERR, "Orb Publish failed\n");
  }
  usleep(500000);
}


  ret = orb_unadvertise(afd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Orb Unadvertise failed.\n");
  }
rd_err:
  close(fd);
error:
  return 0;
}


int main(int argc, FAR char *argv[])
{
  int ret;

  printf("[mpu6500] Starting task.\n");
  if (g_mpu6500_daemon_started)
  {
    printf("[mpu6500] Task already started.\n");
  return EXIT_SUCCESS;
  }

  ret = task_create("mpu6500_daemon",SCHED_PRIORITY_DEFAULT,
                    CONFIG_CUSTOM_APPS_MPU6500_STACKSIZE, mpu6500_daemon,
                    NULL);

  if (ret < 0)
  {
    int errcode = errno;
    printf("[mpu6500] ERROR: Failed to start mpu6500_dameon: %d\n",
           errcode);
    return EXIT_FAILURE;
  }

  printf("[mpu6500] mpu6500_daemon started\n");
  return EXIT_SUCCESS;
  
  
}