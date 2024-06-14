/****************************************************************************
 * apps/custom_apps/spi_test/spi_test_main.c
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
#include <sched.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <time.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/lis3mdl.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include <nuttx/sensors/ioctl.h>

#include "adc.h"
// #include "write_to_file.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_LOW_MASK 0xFF00
#define REG_HIGH_MASK 0x00FF
#define MPU6050_FS_SEL 32.8f
#define MPU6050_AFS_SEL 4096.0f
#define DEVNAME_SIZE 32


#define IOCTL_MODE  1
// #define READ_MODE   1

#define MAX_CHANNELS  12                // maximum channel for external ADC

#define DELAY_ADC 4000
#define ADC_DELAY 6000
#define HK_DELAY 9000
#define BEACON_DELAY 180
/****************************************************************************
 * Public Functions
 ****************************************************************************/
// struct 

/*
Defing work structures for work_queue thread
*/

static struct work_s work_adc;
static struct work_s work_hk;
static struct work_s work_internal_adc;

/*
Declaring structure necessary for collecting HK data
*/
struct sensor_accel imu_acc_data;
struct sensor_gyro imu_gyro_data;
struct mpu6500_imu_msg raw_imu;
struct mpu6500_imu_msg
{
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t temp;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;
};


/****************************************************************************
 * spi_test_main
 ****************************************************************************/



static void adc_devpath(FAR struct adc_state_s *adc, FAR const char *devpath);
static void int_adc_main();
static void ext_adc_main();
static void adc_main(){
  int_adc_main();
  ext_adc_main();
  // work_queue(HPWORK, &work_adc, adc_main, NULL, MSEC2TICK(DELAY_ADC));

}
static void collect_hk();
// void read_mpu6050(int fd, struct sensor_accel *acc_data, struct sensor_gyro *gyro_data, struct mpu6500_imu_msg *raw_imu);
// void read_lis3mdl(int fd_mag, struct mpu6500_imu_msg *raw_imu, int16_t mag_data[4]);

struct sensor_accel imu_acc_data;
struct sensor_gyro imu_gyro_data;
struct mpu6500_imu_msg raw_imu;

struct satellite_health_s {
	uint64_t timestamp;
	float accl_x;
	float accl_y;
	float accl_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
	int16_t temp_x;
	int16_t temp_x1;
	int16_t temp_y;
	int16_t temp_y1;
	int16_t temp_z;
	int16_t temp_z1;
	int16_t temp_bpb;
	int16_t temp_obc;
	int16_t temp_com;
	int16_t temp_batt;
	uint16_t batt_volt;
	int16_t sol_p1_v;
	int16_t sol_p2_v;
	int16_t sol_p3_v;
	int16_t sol_p4_v;
	int16_t sol_p5_v;
	int16_t sol_t_v;
	int16_t raw_v;
	int16_t sol_p1_c;
	int16_t sol_p2_c;
	int16_t sol_p3_c;
	int16_t sol_p4_c;
	int16_t sol_p5_c;
	int16_t sol_t_c;
	int16_t rst_3v3_c;
	int16_t raw_c;
	uint16_t v3_main_c;
	uint16_t v3_com_c;
	uint16_t v3_2_c;
	uint16_t v5_c;
	uint16_t unreg_c;
	uint16_t v4_c;
	int16_t batt_c;
	uint8_t rsv_cmd;
	uint8_t ant_dep_stat;
	uint8_t ul_state;
	uint8_t oper_mode;
	uint8_t msn_flag;
	uint8_t rsv_flag;
	uint8_t kill_switch;
};
/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct adc_state_s g_adcstate;

int elapsed =0;
int required  = 10;

struct adc_msg_s sample[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE];
size_t readsize;
ssize_t nbytes;
int fd;
int errval = 0;
int ret;
char adc[20];
uint8_t raw_data[2] = {'\0'};
uint16_t combined_data[MAX_CHANNELS] = {'\0'};

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/
void test1(){
  printf("reached test 1 \n");
 work_queue(HPWORK, &work_adc,test1, NULL,MSEC2TICK(3000));

}

void test2(){
  printf("reached test 2\n");
 work_queue(HPWORK, &work_adc,test2, NULL,MSEC2TICK(6000));

}
int main(int argc, FAR char *argv[])
{
  work_queue(HPWORK, &work_hk, collect_hk, NULL, MSEC2TICK(HK_DELAY));
//  work_queue(HPWORK, &work_adc,test1, NULL,MSEC2TICK(3000));
//  work_queue(HPWORK, &work_hk,test2, NULL,MSEC2TICK(6000));
// int ret;
//   ret = task_create("ADC",10,2048,int_adc_main,NULL);
//  if(ret <0){
//   printf("Error creatinf task ret:%d",ret);
//  }
//  else{
//   printf("Task created");
//  }
//  usleep(5000);
//   ret = task_create("ADC",10,2048,ext_adc_main,NULL);
//  if(ret <0){
//   printf("Error creatinf task ret:%d",ret);
//  }
//  else{
//   printf("Task created");
//  }
  printf("Custom_apps CUBUS external ADC: %d\n", CONFIG_CUSTOM_APPS_CUBUS_USE_EXT_ADC);
  #if defined(CONFIG_CUSTOM_APPS_CUBUS_USE_EXT_ADC)
  //  work_queue(HPWORK, &work_adc, ext_adc_main, NULL, MSEC2TICK(DELAY_ADC+2000));

    // ext_adc_main();
  #endif  //CONFIG_CUSTOM_APPS_CUBUS_USE_EXT_ADC

  #ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC
    // int_adc_main();
    work_queue(HPWORK, &work_internal_adc, adc_main, NULL, MSEC2TICK(DELAY_ADC));
  #endif  //CUSTOM_APPS_CUBUS_USE_INT_ADC
  //  write_to_file("/mnt/fs/mfm/mtd_mainstorage","/numbers.txt","\nstraiht from main app\n");
  
  // #ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_HK
        // #endif
  
  return 0;
}



/****************************************************************************
 * Name: adc_devpath
 ****************************************************************************/

static void adc_devpath(FAR struct adc_state_s *adc, FAR const char *devpath)
{
  /* Get rid of any old device path */

  if (adc->devpath)
    {
      free(adc->devpath);
    }

  /* Then set-up the new device path by copying the string */

  adc->devpath = strdup(devpath);
}

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

static void int_adc_main(void)
{
  UNUSED(ret);

  /* Check if we have initialized */

  if (!g_adcstate.initialized)
    {
      /* Initialization of the ADC hardware must be performed by
       * board-specific logic prior to running this test.
       */

      /* Set the default values */

      adc_devpath(&g_adcstate, CONFIG_CUSTOM_APPS_CUBUS_INT_ADC_DEVPATH);

      g_adcstate.initialized = true;
    }

  g_adcstate.count = CONFIG_CUSTOM_APPS_CUBUS_INT_ADC_NSAMPLES;

  /* Parse the command line */

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

  printf("adc_main: g_adcstate.count: %d\n", g_adcstate.count);

  /* Open the ADC device for reading */

  printf("adc_main: Hardware initialized. Opening the ADC device: %s\n",
         g_adcstate.devpath);
  
  fd = open(CONFIG_CUSTOM_APPS_CUBUS_INT_ADC_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      printf("adc_main: open %s failed: %d\n", g_adcstate.devpath, errno);
      errval = 2;
      goto errout;
    }
    elapsed = 0;
    while(elapsed < required){
        usleep(1000);
        elapsed++;
    }
  /* Now loop the appropriate number of times, displaying the collected
   * ADC samples.
   */
  // UNUSED(elapsed);
  // UNUSED(required);
  for (int i =0; i <g_adcstate.count+1;i++ )
    {
      /* Flush any output before the loop entered or from the previous pass
       * through the loop.
       */
      elapsed = 0;
      fflush(stdout);

#ifdef CONFIG_CUSTOM_APPS_CUBUS_INT_ADC_SWTRIG
      /* Issue the software trigger to start ADC conversion */

      ret = ioctl(fd, ANIOC_TRIGGER, 0);
      if (ret < 0)
        {
          int errcode = errno;
          printf("adc_main: ANIOC_TRIGGER ioctl failed: %d\n", errcode);
        }
#endif

      /* Read up to CONFIG_CUSTOM_APPS_ADC_GROUPSIZE samples */

      readsize = CONFIG_CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE * sizeof(struct adc_msg_s);
      nbytes = read(fd, sample, readsize);

      printf(" Readsize: %d \n nbytes: %d\n CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE : %d \n ADCSTATE READCOUNT: %d \r\n",readsize, nbytes, CONFIG_CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE, g_adcstate.count);

      // if(nbytes < 40){
      //   printf("nbytes is less than 40. Reading again \n");
      //   nbytes = read(fd, sample, readsize);
      // }
      /* Handle unexpected return values */

      if (nbytes < 0)
        {
          errval = errno;
          if (errval != EINTR)
            {
              printf("adc_main: read %s failed: %d\n",
                     g_adcstate.devpath, errval);
              errval = 3;
              goto errout_with_dev;
            }

          printf("adc_main: Interrupted read...\n");
        }
      else if (nbytes == 0)
        {
          printf("adc_main: No data read, Ignoring\n");
        }

      /* Print the sample data on successful return */

      else
        {
          int nsamples = nbytes / sizeof(struct adc_msg_s);
          if (nsamples * sizeof(struct adc_msg_s) != nbytes)
            {
              printf("adc_main: read size=%ld is not a multiple of "
                     "sample size=%d, Ignoring\n",
                     (long)nbytes, sizeof(struct adc_msg_s));
            }
          else
            {
              printf("Sample:\n");
              for (int i = 0; i < nsamples; i++)
                {
                  printf("%d: channel: %d value: %" PRId32 "\n",
                         i + 1, sample[i].am_channel, sample[i].am_data);
                  sprintf(adc,"%d",sample[i].am_data);
                  // write_to_file("/mnt/fs/mfm/mtd_mainstorage","/adc.txt",adc);
                }
            }
        }

      if (g_adcstate.count && --g_adcstate.count <= 0)
        {
          break;
        }
    }
  close(fd);
  printf("end of int_adcc\n***********\n");
    work_queue(HPWORK, &work_internal_adc, int_adc_main, NULL, MSEC2TICK(3000));
  return OK;

  /* Error exits */

errout_with_dev:
  close(fd);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  close(fd);
  return errval;
}


void read_lis3mdl(int fd_mag, struct mpu6500_imu_msg *raw_imu, int16_t mag_data[4])
{
  assert(fd_mag >= 0);
  int data_size = read(fd_mag, mag_data, 8);
  if (data_size > 0)
  {
    printf("read sensor data from Mag. Len %i\n", data_size);
    raw_imu->mag_x = mag_data[0];
    raw_imu->mag_y = mag_data[1];
    raw_imu->mag_z = mag_data[2];
    printf("Magnetometer func: x:%d y:%d z:%d\n", mag_data[0], mag_data[1], mag_data[2]);
  }
  else
  {
    printf("Failed to read from sensor.\n");
  }
}

void read_mpu6050(int fd, struct sensor_accel *acc_data, struct sensor_gyro *gyro_data, struct mpu6500_imu_msg *raw_imu)
{
  int16_t raw_data[7];
  memset(raw_imu, 0, sizeof(struct mpu6500_imu_msg));
  ret = read(fd, raw_data, sizeof(raw_data));
  if (ret != sizeof(raw_data))
  {
    printf("Failed to read accelerometer data\n");
  }
  else
  {
    raw_imu->acc_x = ((raw_data[0] & REG_HIGH_MASK) << 8) + ((raw_data[0] & REG_LOW_MASK) >> 8);
    raw_imu->acc_y = ((raw_data[1] & REG_HIGH_MASK) << 8) + ((raw_data[1] & REG_LOW_MASK) >> 8);
    raw_imu->acc_z = ((raw_data[2] & REG_HIGH_MASK) << 8) + ((raw_data[2] & REG_LOW_MASK) >> 8);
    raw_imu->gyro_x = ((raw_data[4] & REG_HIGH_MASK) << 8) + ((raw_data[4] & REG_LOW_MASK) >> 8);
    raw_imu->gyro_y = ((raw_data[5] & REG_HIGH_MASK) << 8) + ((raw_data[5] & REG_LOW_MASK) >> 8);
    raw_imu->gyro_z = ((raw_data[6] & REG_HIGH_MASK) << 8) + ((raw_data[6] & REG_LOW_MASK) >> 8);
  }

  acc_data->x = raw_imu->acc_x / MPU6050_AFS_SEL;
  acc_data->y = raw_imu->acc_y / MPU6050_AFS_SEL;
  acc_data->z = raw_imu->acc_z / MPU6050_AFS_SEL;

  gyro_data->x = raw_imu->gyro_x / MPU6050_FS_SEL;
  gyro_data->y = raw_imu->gyro_y / MPU6050_FS_SEL;
  gyro_data->z = raw_imu->gyro_z / MPU6050_FS_SEL;
}

static void collect_hk()
{
  int opt;
  float acq_period = CONFIG_EXAMPLES_SENSOR_FUSION_SAMPLE_RATE / 1000.0f;
  printf("Sensor Fusion example\n");
  printf("Sample Rate: %.2f Hz\n", 1.0 / acq_period);
  printf("got inside sensor_work");
  int fd, fd_mag;

  int16_t mag_data[4];

  fd = open("/dev/mpu6500", O_RDONLY);
  if (fd < 0)
  {
    printf("Failed to open mpu6500\n");
    return;
  }

  fd_mag = open("/dev/mag0", O_RDONLY);
  if (fd_mag < 0)
  {
    printf("Failed to open magnetometer\n");
    close(fd);
    return;
  }

  read_mpu6050(fd, &imu_acc_data, &imu_gyro_data, &raw_imu);
  read_lis3mdl(fd_mag, &raw_imu, mag_data);

  printf("Timestamp: %f  Temperature: %f\n"
         "Accelerometer X: %f | Y: %f | Z: %f\n"
         "Gyroscope X: %f | Y: %f | Z: %f\n"
         "Magnetometer X: %f | Y: %f | Z: %f\n",
         imu_acc_data.timestamp, imu_acc_data.temperature,
         imu_acc_data.x, imu_acc_data.y, imu_acc_data.z,
         imu_gyro_data.x, imu_gyro_data.y, imu_gyro_data.z,
         raw_imu.mag_x, raw_imu.mag_y, raw_imu.mag_z);

  close(fd);
  close(fd_mag);
  printf("end of hk\n");

  work_queue(HPWORK, &work_hk, collect_hk, NULL, MSEC2TICK(HK_DELAY));
  
}

static void ext_adc_main(){
   printf("Going to Test the External ADC\n");
  fd = open(EXT_ADC_PATH, O_RDONLY);
  if(fd < 0){
    printf("Unable to open external ADC driver\n");
    return -1;
  }
  printf("opened external ADC driver successfully\n Setting Manual Select mode...\n");

  /* Get the set of BUTTONs supported */
  ret = ioctl(fd, ANIOC_ADC_MANUAL_SELECT, NULL);
  usleep(10);

  printf("Setting ADC Select mode ... \n");
  ret = ioctl(fd, ANIOC_ADC_AUTO_2_SELECT, NULL);
  usleep(1000);

  printf("Setting ADC Program mode ...\n");
  ret = ioctl(fd, ANIOC_ADC_AUTO_2_PROGRAM, NULL);
  usleep(1000);

  #ifdef IOCTL_MODE
  for(int i=0;i<MAX_CHANNELS;i++){
    printf("Reading data from ADC %i \n", i);
    ioctl(fd, ANIOC_ADC_AUTO_2_SELECT_READ,raw_data);
    combined_data[i] = raw_data[0] << 8 | raw_data[1];
    printf("Raw data: %x \n",combined_data[i]);
    // usleep(100);
  }

  #else //ifndef IOCTL MODE
  for (int i=0;i<MAX_CHANNELS;i++){
    int ret1 = read(fd, &raw_data, 2);
    if(ret1<0){
      printf("Data not received from ADC");
      return -1;
    }
    printf("No of Bytes available: %d",ret1);
    combined_data[i] = raw_data[0] << 8 | raw_data[1];
    printf("\n\n\n");
  }
  #endif  //IOCTL MODE
  printf("end of ext_adcc\n");
 //  work_queue(HPWORK, &work_adc, ext_adc_main, NULL, MSEC2TICK(DELAY_ADC+2000));

  return 0;
}