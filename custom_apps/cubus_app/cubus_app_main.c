/****************************************************************************
 * apps/examples/spi_test/spi_test_main.c
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

#include "cubus_app_main.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_devpath
 ****************************************************************************/
#if defined(CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1) || defined(CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3)
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
#endif
int int_adc_main();

void print_satellite_health_data();

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct adc_state_s g_adcstate1;
static struct adc_state_s g_adcstate3;

int elapsed = 0;
int required  = 10;

int ret;
int i = 0; 
char buffer[255] = {'\0'};
struct file file_p;

#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1
  int_adc_config_s adc1_config;
  struct adc_msg_s int_adc1_sample[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_GROUPSIZE];
#endif

#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3
  int_adc_config_s adc3_config;
  struct adc_msg_s int_adc3_sample[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_GROUPSIZE];
#endif

#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_EXT_ADC
  ext_adc_config_s ext_adc_config;
#endif

uint8_t raw_data[2] = {'\0'};
uint16_t combined_data[EXT_ADC_MAX_CHANNELS] = {'\0'};
float processed_data_ext_adc[EXT_ADC_MAX_CHANNELS] = {'\0'};

satellite_health_s sat_health = {'\0'};
ext_adc_s ext_adc_data[EXT_ADC_MAX_CHANNELS];

/****************************************************************************
 * Name: main
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
  
  #if defined(CONFIG_CUSTOM_APPS_CUBUS_USE_EXT_ADC)
  ext_adc_main();
  #endif  //CONFIG_CUSTOM_APPS_CUBUS_USE_EXT_ADC

  #ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1
  read_int_adc1();
  #endif  //CUSTOM_APPS_CUBUS_USE_INT_ADC

  #ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3
  read_int_adc3();
  #endif

  make_satellite_health();

  print_satellite_health_data(&sat_health);
  const char file_name[] = {"/str.txt"};
  syslog(LOG_INFO, "Opening a file inside %s.\n", MFM_MAIN_STRPATH);
  char path[65];
  sprintf(path, "%s%s", MFM_MAIN_STRPATH,file_name);
  syslog(LOG_INFO, "file path: %s\n", path);
  // int fd = open(path, O_CREAT | O_RDWR | O_APPEND);
  int fd = file_open(&file_p, path, O_RDWR | O_CREAT | O_APPEND);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Error opening file in mainstorage of MFM.\n");
  } else {
    ssize_t bytes_written = file_write(&file_p, &sat_health, sizeof(sat_health));
    if  (bytes_written > 0)
    {
      syslog(LOG_INFO, "Falsh Write Successful.\nData Len: %d.\n", bytes_written);
      file_close(&file_p);
    } else {
      syslog(LOG_INFO, "Write Failure.\n");
    }
    file_syncfs(&file_p);
    file_close(&file_p);
  }

  syslog(LOG_INFO, "\n\n\nReading stored data from flash mem.\n");
  fd = file_open(&file_p, path, O_RDONLY);
  if(fd < 0)
  {
    syslog(LOG_ERR, "Error opening file in mainstorage of MFM.\n");
  }else {
    satellite_health_s health_read;
    ssize_t bytes_read;
    int off;
    off = file_seek(&file_p,0, SEEK_SET);
    while(!(off < 0) ) 
    {
    bytes_read = file_read(&file_p, &health_read, sizeof(health_read));
    syslog(LOG_INFO,"Read %i bytes from flash. to %i size str.\n", bytes_read, sizeof(health_read));
    if(!bytes_read)
    {
      syslog(LOG_ERR, "Read error\n");
      goto ERR_FILE_CLOSE;
    }
    print_satellite_health_data(&health_read);
    // off = file_seek(&file_p, sizeof(health_read), SEEK_CUR);
    // if(!off)
    // {
    //   syslog("Error in offsetting %i bytes.\n", sizeof(health_read));
    //   goto ERR_FILE_CLOSE;
    // } else {
    //   syslog(LOG_INFO, "pointer moved %i forward.\n", off);
    // }
    }
  }

  return 0;

ERR_FILE_CLOSE:
  file_close(&file_p);
  return 0;

}

/*
* TODO: Add number of samples and append in a two dimensional array and return the average value
*/

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1
int read_int_adc1(){

  UNUSED(ret);

  /* Check if we have initialized */
  if (!g_adcstate1.initialized)
    {
      /* Initialization of the ADC hardware must be performed by
       * board-specific logic prior to running this test.
       */

      /* Set the default values */

      adc_devpath(&g_adcstate1, CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_DEVPATH);

      g_adcstate1.initialized = true;
    }

  g_adcstate1.count = CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_NSAMPLES;

  /* Parse the command line */

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

  printf("adc_main: g_adcstate.count: %d\n", g_adcstate1.count);

  /* Open the ADC device for reading */

  printf("adc_main: Hardware initialized. Opening the ADC device: %s\n",
         g_adcstate1.devpath);
  
  /* Opening internal ADC1 */
  adc1_config.fd = open(CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_DEVPATH, O_RDONLY);
  if (adc1_config.fd < 0)
    {
      printf("adc_main: open %s failed: %d\n", g_adcstate1.devpath, errno);
      adc1_config.errval = 2;
      goto errout;
    }
    elapsed = 0;
    while(elapsed<required){
        usleep(1);
        elapsed++;
    }
  /* Now loop the appropriate number of times, displaying the collected
   * ADC samples.
   */
  // UNUSED(elapsed);
  // UNUSED(required);
  for (int j =0; j<1; j++)
    {
      /* Flush any output before the loop entered or from the previous pass
       * through the loop.
       */
      elapsed = 0;
      fflush(stdout);

#ifdef CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_SWTRIG
      /* Issue the software trigger to start ADC conversion */

      ret = ioctl(adc1_config.fd, ANIOC_TRIGGER, 0);
      if (ret < 0)
        {
          int errcode = errno;
          printf("adc_main: ANIOC_TRIGGER ioctl failed: %d\n", errcode);
        }
#endif

      /* Read up to CONFIG_CUSTOM_APPS_ADC_GROUPSIZE samples */

      adc1_config.readsize = CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_GROUPSIZE * sizeof(struct adc_msg_s);
      adc1_config.nbytes = read(adc1_config.fd, int_adc1_sample, adc1_config.readsize);

      printf("Readsize: %d \n nbytes: %d\n CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE : %d \n ADCSTATE READCOUNT: %d \r\n",adc1_config.readsize, adc1_config.nbytes, CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_GROUPSIZE, g_adcstate1.count);

      /* Handle unexpected return values */
      if (adc1_config.nbytes < 0)
        {
          adc1_config.errval = errno;
          if (adc1_config.errval != EINTR)
            {
              printf("adc_main: read %s failed: %d\n",
                     g_adcstate1.devpath, adc1_config.errval);
              adc1_config.errval = 3;
              goto errout_with_dev;
            }

          printf("adc_main: Interrupted read...\n");
        }
      else if (adc1_config.nbytes == 0)
        {
          printf("adc_main: No data read, Ignoring\n");
        }

      /* Print the sample data on successful return */

      else
        {
          int nsamples = adc1_config.nbytes / sizeof(struct adc_msg_s);
          if (nsamples * sizeof(struct adc_msg_s) != adc1_config.nbytes)
            {
              printf("adc_main: read size=%ld is not a multiple of "
                     "sample size=%d, Ignoring\n",
                     (long)adc1_config.nbytes, sizeof(struct adc_msg_s));
            }
          else
            {
              printf("Sample:\n");
              for (i = 0; i < nsamples; i++)
                {
                  printf("%d: channel: %d value: %" PRId32 "\n",
                         i, int_adc1_sample[i].am_channel, int_adc1_sample[i].am_data);
                }
            }
        }

      if (g_adcstate1.count && --g_adcstate1.count <= 0)
        {
          break;
        }
    }
    

  close(adc1_config.fd);
  return OK;

/* Error exits */
errout_with_dev:
  close(adc1_config.fd);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  close(adc1_config.fd);
  return adc1_config.errval;
}
#endif  //CONFIG_EAMPLES_CUBUS_USE_ADC_1

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3
int read_int_adc3(){

  UNUSED(ret);

  /* Check if we have initialized */
  if (!g_adcstate3.initialized)
    {
      /* Initialization of the ADC hardware must be performed by
       * board-specific logic prior to running this test.
       */

      /* Set the default values */

      adc_devpath(&g_adcstate3, CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_DEVPATH);

      g_adcstate3.initialized = true;
    }

  g_adcstate3.count = CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_NSAMPLES;

  /* Parse the command line */

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

  printf("adc_main: g_adcstate.count: %d\n", g_adcstate3.count);

  /* Open the ADC device for reading */

  printf("adc_main: Hardware initialized. Opening the ADC device: %s\n",
         g_adcstate3.devpath);
  
  /* Opening internal ADC1 */
  adc3_config.fd = open("/dev/adc0", O_RDONLY);
  if (adc3_config.fd < 0)
    {
      printf("adc_main: open %s failed: %d\n", g_adcstate3.devpath, errno);
      adc3_config.errval = 2;
      goto errout;
    }
    elapsed = 0;
    while(elapsed<required){
        usleep(1);
        elapsed++;
    }
  /* Now loop the appropriate number of times, displaying the collected
   * ADC samples.
   */
  // UNUSED(elapsed);
  // UNUSED(required);
  for (int k = 0; k<1; k++)
    {
      /* Flush any output before the loop entered or from the previous pass
       * through the loop.
       */
      elapsed = 0;
      fflush(stdout);

#ifdef CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_SWTRIG
      /* Issue the software trigger to start ADC conversion */

      ret = ioctl(adc3_config.fd, ANIOC_TRIGGER, 0);
      if (ret < 0)
        {
          int errcode = errno;
          printf("adc_main: ANIOC_TRIGGER ioctl failed: %d\n", errcode);
        }
#endif

      /* Read up to CONFIG_CUSTOM_APPS_ADC_GROUPSIZE samples */

      adc3_config.readsize = CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_GROUPSIZE * sizeof(struct adc_msg_s);
      adc3_config.nbytes = read(adc3_config.fd, int_adc3_sample, adc3_config.readsize);

      printf("Readsize: %d \n nbytes: %d\n CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE : %d \n ADCSTATE READCOUNT: %d \r\n",adc3_config.readsize, adc3_config.nbytes, CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_GROUPSIZE, g_adcstate3.count);

      /* Handle unexpected return values */
      if (adc3_config.nbytes < 0)
        {
          adc3_config.errval = errno;
          if (adc3_config.errval != EINTR)
            {
              printf("adc_main: read %s failed: %d\n",
                     g_adcstate3.devpath, adc3_config.errval);
              adc3_config.errval = 3;
              goto errout_with_dev;
            }

          printf("adc_main: Interrupted read...\n");
        }
      else if (adc3_config.nbytes == 0)
        {
          printf("adc_main: No data read, Ignoring\n");
        }

      /* Print the sample data on successful return */
      else
        {
          int nsamples = adc3_config.nbytes / sizeof(struct adc_msg_s);
          if (nsamples * sizeof(struct adc_msg_s) != adc3_config.nbytes)
            {
              printf("adc_main: read size=%ld is not a multiple of "
                     "sample size=%d, Ignoring\n",
                     (long)adc3_config.nbytes, sizeof(struct adc_msg_s));
            }
          else
            {
              printf("Sample:\n");
              for (i = 0; i < nsamples; i++)
                {
                  printf("%d: channel: %d value: %" PRId32 "\n",
                         i , int_adc3_sample[i].am_channel, int_adc3_sample[i].am_data);
                }
            }
        }

      if (g_adcstate3.count && --g_adcstate3.count <= 0)
        {
          break;
        }
    }
    

  close(adc3_config.fd);
  return OK;

/* Error exits */
errout_with_dev:
  close(adc3_config.fd);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  close(adc3_config.fd);
  return adc3_config.errval;
}
#endif //CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_EXT_ADC
int ext_adc_main(){
  
  printf("Going to Test the External ADC\n");
  ext_adc_config.fd = open(EXT_ADC_PATH, O_RDONLY);
  if(ext_adc_config.fd < 0){
    printf("Unable to open external ADC driver\n");
    return -1;
  }
  printf("opened external ADC driver successfully\n");
  /* Get the set of BUTTONs supported */
  ret = ioctl(ext_adc_config.fd, ANIOC_ADC_MANUAL_SELECT, NULL);
  usleep(10);
  ret = ioctl(ext_adc_config.fd, ANIOC_ADC_AUTO_2_SELECT, NULL);
  usleep(1000);
  ret = ioctl(ext_adc_config.fd, ANIOC_ADC_AUTO_2_PROGRAM, NULL);
  usleep(1000);

  #ifdef IOCTL_MODE
  for(int i=0;i<EXT_ADC_MAX_CHANNELS;i++){
    ioctl(ext_adc_config.fd, ANIOC_ADC_AUTO_2_SELECT_READ,raw_data);

    /* Separate Channel ID and corresponding data */

    combined_data[i] = raw_data[0] << 8 | raw_data[1];
    ext_adc_data[i].chan = combined_data[i] >> 12 & 0xff;
    ext_adc_data[i].raw_data = (int16_t)combined_data[i] & 0x0fff;
    ext_adc_data[i].raw_data = (2.5 * ext_adc_data[i].raw_data)/4095;
    // sprintf(buffer,"%.2f",ext_adc_data[i].raw_data);
    printf("Channel: %d   value: %d \r\n",ext_adc_data[i].chan, (int)ext_adc_data[i].raw_data);

    if(i == 9){ //battery temperature channel
      float res = (ext_adc_data[i].raw_data * 10000) / (2.5 - ext_adc_data[i].raw_data);
      float tempK = (3976 * 298) / (3976 - (298 * log(10000 / res)));
      ext_adc_data[i].processed_data = (tempK - 273) * 100;
    }else if(i == 8 || i == 10 || i == 11){ //other temperature channels (antenna, bpb, z panel)
      float root = sqrtf((5.506 * 5.506) + (4 * 0.00176 * (870.6 - (ext_adc_data[i].raw_data * 1000))));
			ext_adc_data[i].processed_data = ((((5.506 - (root) / (2 * (-0.00176))) + 30)) * 100);
    }else{
      ext_adc_data[i].processed_data = (ext_adc_data[i].raw_data * (1100 + 931)) / 931;
    }
  }

  #else //ifndef IOCTL MODE
  for (int i=0;i<EXT_ADC_EXT_ADC_MAX_CHANNELS;i++){
    int ret1 = read(ext_adc_config.fd, &raw_data, 2);
    if(ret1<0){
      printf("Data not received from ADC");
      return -1;
    }
    printf("No of Bytes available: %d",ret1);
    combined_data[i] = raw_data[0] << 8 | raw_data[1];
    printf("\n\n\n");
  }
  #endif  //IOCTL MODE
}
#endif


/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

void make_satellite_health(){


  float int_adc1_temp[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_GROUPSIZE] = {'\0'};  
  int_adc1_data_convert(int_adc1_temp);

  float int_adc3_temp[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_GROUPSIZE] = {'\0'};
  int_adc3_data_convert(int_adc3_temp);

  /* External ADC data */
  sat_health.sol_t_v = (int16_t)ext_adc_data[0].processed_data;
  sat_health.raw_v = (int16_t)ext_adc_data[1].processed_data;
  sat_health.sol_p5_v = (int16_t)ext_adc_data[2].processed_data;
  sat_health.sol_p4_v = (int16_t)ext_adc_data[3].processed_data;
  sat_health.sol_p3_v = (int16_t)ext_adc_data[4].processed_data;
  sat_health.sol_p1_v = (int16_t)ext_adc_data[5].processed_data;
  sat_health.sol_p2_v = (int16_t)ext_adc_data[6].processed_data;

  sat_health.ant_temp_out = (int16_t)ext_adc_data[8].processed_data;
  // sat_health.temp_batt = (int16_t)ext_adc_data[9].processed_data;
  sat_health.temp_bpb = (int16_t)ext_adc_data[10].processed_data;
  sat_health.temp_z = (int16_t)ext_adc_data[11].processed_data;


  /* Internal ADC1 data */
  sat_health.batt_c = (int16_t)int_adc1_temp[9];
  sat_health.sol_t_c = (int16_t)int_adc1_temp[10];
  sat_health.raw_c = (int16_t)int_adc1_temp[11];

  sat_health.unreg_c = (int16_t)int_adc1_temp[0];
  sat_health.v3_main_c = (int16_t)int_adc1_temp[1];
  sat_health.v3_com_c = (int16_t)int_adc1_temp[2];
  sat_health.v5_c = (int16_t)int_adc1_temp[3];

  sat_health.batt_volt = (int16_t)int_adc1_temp[4];

  sat_health.sol_p1_c = (int16_t)int_adc1_temp[5];
  sat_health.v3_2_c = (int16_t)int_adc1_temp[6];
  sat_health.sol_p4_c = (int16_t)int_adc1_temp[7];
  sat_health.sol_p5_c = (int16_t)int_adc1_temp[8];

  sat_health.sol_p2_c = (int16_t)int_adc1_temp[12];
  sat_health.sol_p3_c = (int16_t)int_adc1_temp[13];

  /* internal adc2 data*/
  sat_health.v4_c = (int16_t)int_adc3_temp[0];
}

/*
*/
void print_satellite_health_data(satellite_health_s *sat_health){
  printf(" *******************************************\r\n");
  printf(" |   Solar Panel 1 Voltage: \t %d \t|\r\n",sat_health->sol_p1_v);
  printf(" |   Solar Panel 2 Voltage: \t %d \t|\r\n",sat_health->sol_p2_v);
  printf(" |   Solar Panel 3 Voltage: \t %d \t|\r\n",sat_health->sol_p3_v);
  printf(" |   Solar Panel 4 Voltage: \t %d \t|\r\n",sat_health->sol_p4_v);
  printf(" |   Solar Panel 5 Voltage: \t %d \t|\r\n",sat_health->sol_p5_v);
  printf(" |   Solar Panel T Voltage: \t %d \t|\r\n", sat_health->sol_t_v);
  printf(" |--------------------------------------|\r\n");
  printf(" |   Solar Panel 1 Current: \t %d \t|\r\n",sat_health->sol_p1_c);
  printf(" |   Solar Panel 2 Current: \t %d \t|\r\n",sat_health->sol_p2_c);
  printf(" |   Solar Panel 3 Current: \t %d \t|\r\n",sat_health->sol_p3_c);
  printf(" |   Solar Panel 4 Current: \t %d \t|\r\n",sat_health->sol_p4_c);
  printf(" |   Solar Panel 5 Current: \t %d \t|\r\n",sat_health->sol_p5_c);
  printf(" |   Solar Panel T Current: \t %d \t|\r\n",sat_health->sol_t_c);
  printf(" |--------------------------------------|\r\n");
  printf(" |   Unreg Line Current:    \t %d \t|\r\n",sat_health->unreg_c);
  printf(" |   Main 3v3 Current:      \t %d \t|\r\n",sat_health->v3_main_c);
  printf(" |   COM 3v3 Current:       \t %d \t|\r\n",sat_health->v3_com_c);
  printf(" |   5 Volts line Current:  \t %d \t|\r\n", sat_health->v5_c);
  printf(" |   3v3 2 line Current:    \t %d \t|\r\n", sat_health->v3_2_c); 
  printf(" |--------------------------------------|\r\n");
  printf(" |   Raw Current:           \t %d \t|\r\n", sat_health->raw_c);
  printf(" |   Raw Voltage:           \t %d \t|\r\n", sat_health->raw_v);
  printf(" |--------------------------------------|\r\n");
  printf(" |   Battery Total Voltage: \t %d \t|\r\n",sat_health->batt_volt);
  printf(" |   Battery Total Current: \t %d \t|\r\n", sat_health->batt_c);
  printf(" |   Battery Temperature:   \t %d \t|\r\n", sat_health->temp_batt);
  printf(" *********************************************\r\n");
}

/****************************************************************************
 * Name: int_adc1_data_convert
 * 
 * Parameters: *temp_buff -- pointer buffer where the data is stored after conversion
 * 
 * Details:
 *   ADC_SUP -- 17th channel (doesn't have pinout) data is ADC_SUPP on vrefint 
 *   Firstly, raw data is converted by using formula: channel_data * vrefint_data / 4095
 *   Then data is converted according to current/voltage sensors data. LMP8640 is used to convert all current data except: raw_current, solar_total_current, and battery_current
 *    
 *    except batt_mon, everything is current data only ... 
 * Channels and Corresponding data:
 *  
 ****************************************************************************/
#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1
void int_adc1_data_convert(float *temp_buff){

  // float temp_buff[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_NSAMPLES];
  float ADC_SUP = 1.2 * 4095/(int_adc1_sample[14].am_data);

  for(int i=0;i<CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_GROUPSIZE;i++){
    temp_buff[i] = (int_adc1_sample[i].am_data * 3.3 / 4095); //right now, using 1.2 as vrefint channel data... converting all the data to their respective voltages
    if(i == 4)
    {
      temp_buff[i] = (temp_buff[i] * (1100 + 931)) / 931;  //this is for battery monitor (voltage data, no need for conversion)
    }
    else if(i == 9 || i == 10 || i == 11) //this one is for battery current, solar panel total current and raw current respectively
    { 
      temp_buff[i] = ((temp_buff[i] - 1.65) / SENS_TMCS) * 1000;
    }
    else
    {
      //all current sensors use lmp8640 current sensor ... 
      temp_buff[i] = (temp_buff[i]/(2*RES_LMP8640*GAIN_LMP8640))*1000;
    }
  }
}
#endif

/****************************************************************************
 * Name: int_adc3_data_convert
 * 
 * Parameters:  *temp_buff -- float pointer to the data that is converted 
 * 
 * Details:
 *    only one channel data is converted by internal adc 3 i.e. 4V_I 
 *    it uses LMP8640 current sensor, so same formula given above is used ...
 ****************************************************************************/
#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3
void int_adc3_data_convert(float *temp_buff_1){
  for(int i=0;i<CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_GROUPSIZE;i++){
    temp_buff_1[i] = (float)int_adc3_sample[0].am_data * 3.3 / 4095;
    temp_buff_1[i] = (float)(temp_buff_1[i]/(2*RES_LMP8640*GAIN_LMP8640))*1000;
  }
}
#endif



