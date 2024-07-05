/****************************************************************************
 * apps/examples/adc/adc_main.c
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
#include "adc_main.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static bool g_adc_daemon_started;
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 * Name: adc_help
 ****************************************************************************/

static void adc_help(FAR struct adc_state_s *adc)
{
  printf("Usage: adc [OPTIONS]\n");
  printf("\nArguments are \"sticky\".  "
         "For example, once the ADC device is\n");
  printf("specified, that device will be re-used until it is changed.\n");
  printf("\n\"sticky\" OPTIONS include:\n");
  printf("  [-p devpath] selects the ADC device.  "
         "Default: %s Current: %s\n",
         CONFIG_CUSTOM_APPS_INT_ADC1_DEVPATH,
         adc->devpath ? adc->devpath : "NONE");
  printf("  [-n count] selects the samples to collect.  "
         "Default: 1 Current: %d\n",
         adc->count);
  printf("  [-h] shows this message and exits\n");
}

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

static int arg_string(FAR char **arg, FAR char **value)
{
  FAR char *ptr = *arg;

  if (ptr[2] == '\0')
  {
    *value = arg[1];
    return 2;
  }
  else
  {
    *value = &ptr[2];
    return 1;
  }
}

/****************************************************************************
 * Name: arg_decimal
 ****************************************************************************/

static int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;

  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

static void parse_args(FAR struct adc_state_s *adc1, FAR struct adc_state_s *adc3, int argc,
                       FAR char **argv)
{
  FAR char *ptr;
  FAR char *str;
  long value;
  int index;
  int nargs;

  for (index = 1; index < argc;)
  {
    ptr = argv[index];
    if (ptr[0] != '-')
    {
      printf("Invalid options format: %s\n", ptr);
      exit(0);
    }

    switch (ptr[1])
    {
    case 'n':
      nargs = arg_decimal(&argv[index], &value);
      if (value < 0)
      {
        printf("Count must be non-negative: %ld\n", value);
        exit(1);
      }

      adc1->count = (uint32_t)value;
      index += nargs;
      break;

    case 'p':
      nargs = arg_string(&argv[index], &str);
      adc_devpath(adc1, str);
      index += nargs;
      break;

    case 'h':
      adc_help(adc1);
      exit(0);

    default:
      printf("Unsupported option: %s\n", ptr);
      adc_help(adc1);
      exit(1);
    }
  }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/


int read_int_adc(FAR struct adc_state_s *g_adcstate,
                 FAR struct int_adc_config_s *adc_config,
                 FAR struct adc_msg_s* int_adc_sample, int sample_count)
{
  int elapsed = 0;
  int required = 10;
  int ret;
  /* Open the ADC device for reading */

  syslog(LOG_INFO, "[adc_main]: Hardware initialized. Opening the ADC device: %s\n",
         g_adcstate->devpath);

  /* Opening internal ADC1 */
  adc_config->fd = open(g_adcstate->devpath, O_RDONLY);
  if (adc_config->fd < 0)
  {
    syslog(LOG_ERR, "[adc_main]: open %s failed: %d\n", g_adcstate->devpath, errno);
    adc_config->errval = 2;
    goto errout;
  }
  elapsed = 0;
  while (elapsed < required)
  {
    usleep(1);
    elapsed++;
  }
  /* Now loop the appropriate number of times, displaying the collected
   * ADC samples.
   */
  // UNUSED(elapsed);
  // UNUSED(required);
  for (int j = 0; j < sample_count; j++)
  {
    /* Flush any output before the loop entered or from the previous pass
     * through the loop.
     */
    elapsed = 0;
    fflush(stdout);

#if defined(CONFIG_CUSTOM_APPS_INT_ADC1_SWTRIG) | defined(CONFIG_CUSTOM_APPS_INT_ADC3_SWTRIG)
    /* Issue the software trigger to start ADC conversion */

    ret = ioctl(adc_config->fd, ANIOC_TRIGGER, 0);
    if (ret < 0)
    {
      int errcode = errno;
      syslog(LOG_ERR, "[adc_main]: ANIOC_TRIGGER ioctl failed: %d\n", errcode);
    }
#endif

    /* Read up to CONFIG_CUSTOM_APPS_ADC_GROUPSIZE samples */
    adc_config->nbytes = read(adc_config->fd, int_adc_sample, adc_config->readsize);

    // syslog(LOG_INFO, "Readsize: %d \n nbytes: %d\n CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE :"
    //                  " %d \n ADCSTATE READCOUNT: %d \r\n",
    //        adc_config->readsize, adc_config->nbytes,
    //        adc_config->readsize / (sizeof(struct adc_msg_s)), g_adcstate->count);

    /* Handle unexpected return values */
    if (adc_config->nbytes < 0)
    {
      adc_config->errval = errno;
      if (adc_config->errval != EINTR)
      {
        syslog(LOG_ERR, "[adc_main]: read %s failed: %d\n",
               g_adcstate->devpath, adc_config->errval);
        adc_config->errval = 3;
        goto errout_with_dev;
      }

      syslog(LOG_ERR, "[adc_main]: Interrupted read...\n");
    }
    else if (adc_config->nbytes == 0)
    {
      syslog(LOG_WARNING, "[adc_main]: No data read, Ignoring\n");
    }

    /* Print the sample data on successful return */

    else
    {
      int nsamples = adc_config->readsize / sizeof(struct adc_msg_s);
      if (nsamples * sizeof(struct adc_msg_s) != adc_config->nbytes)
      {
        syslog(LOG_ERR, "adc_main: read size=%ld is not a multiple of "
                        "sample size=%d, Ignoring\n",
               (long)adc_config->nbytes, sizeof(struct adc_msg_s));
      }
      else
      {
        // syslog(LOG_INFO, "Sample:\n");
        // for (int i = 0; i < nsamples; i++)
        // {
        //   struct adc_msg_s temp = int_adc_sample[i];
        //   syslog(LOG_INFO, "%d: channel: %d value: %" PRId32 "\n",
        //         i, temp.am_channel, temp.am_data);
        // }
      }
    }

    // if (g_adcstate->count && --g_adcstate->count < 0)
    // {
    //   break;
    // }
  }

  close(adc_config->fd);
  return OK;

/* Error exits */
errout_with_dev:
  close(adc_config->fd);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  close(adc_config->fd);
  return adc_config->errval;
}

#ifdef CONFIG_CUSTOM_APPS_USE_INT_ADC3
// int read_int_adc3()
// {

//   UNUSED(ret);

//   /* Parse the command line */

//   /* If this example is configured as an NX add-on, then limit the number of
//    * samples that we collect before returning.  Otherwise, we never return
//    */

//   syslog(LOG_INFO, "[adc_main]: g_adcstate3.count: %d\n", g_adcstate3.count);

//   /* Open the ADC device for reading */

//   syslog(LOG_INFO, "[adc_main]: Hardware initialized. Opening the ADC device: %s\n",
//          g_adcstate3.devpath);

//   /* Opening internal ADC1 */
//   adc3_config.fd = open("/dev/adc0", O_RDONLY);
//   if (adc3_config.fd < 0)
//   {
//     syslog(LOG_ERR, "[adc_main]: open %s failed: %d\n", g_adcstate3.devpath, errno);
//     adc3_config.errval = 2;
//     goto errout;
//   }
//   elapsed = 0;
//   while (elapsed < required)
//   {
//     usleep(1);
//     elapsed++;
//   }
//   /* Now loop the appropriate number of times, displaying the collected
//    * ADC samples.
//    */
//   // UNUSED(elapsed);
//   // UNUSED(required);
//   for (int k = 0; k < 1; k++)
//   {
//     /* Flush any output before the loop entered or from the previous pass
//      * through the loop.
//      */
//     elapsed = 0;
//     fflush(stdout);

// #ifdef CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_SWTRIG
//     /* Issue the software trigger to start ADC conversion */

//     ret = ioctl(adc3_config.fd, ANIOC_TRIGGER, 0);
//     if (ret < 0)
//     {
//       int errcode = errno;
//       syslog(LOG_ERR, "[adc_main]: ANIOC_TRIGGER ioctl failed: %d\n", errcode);
//     }
// #endif

//     /* Read up to CONFIG_CUSTOM_APPS_ADC_GROUPSIZE samples */

//     adc3_config.nbytes = read(adc3_config.fd, int_adc3_sample, adc3_config.readsize);
//     syslog(LOG_INFO, "Readsize: %d \n nbytes: %d\n CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE : %d \n ADCSTATE READCOUNT: %d \r\n",
//            adc3_config.readsize, adc3_config.nbytes, CONFIG_CUSTOM_APPS__INT_ADC3_GROUPSIZE, g_adcstate3.count);

//     /* Handle unexpected return values */
//     if (adc3_config.nbytes < 0)
//     {
//       adc3_config.errval = errno;
//       if (adc3_config.errval != EINTR)
//       {
//         syslog(LOG_ERR, "adc_main: read %s failed: %d\n",
//                g_adcstate3.devpath, adc3_config.errval);
//         adc3_config.errval = 3;
//         goto errout_with_dev;
//       }

//       syslog(LOG_ERR, "[adc_main]: Interrupted read...\n");
//     }
//     else if (adc3_config.nbytes == 0)
//     {
//       syslog(LOG_WARNING, "[adc_main]: No data read, Ignoring\n");
//     }

//     /* Print the sample data on successful return */
//     else
//     {
//       int nsamples = adc3_config.nbytes / sizeof(struct adc_msg_s);
//       if (nsamples * sizeof(struct adc_msg_s) != adc3_config.nbytes)
//       {
//         syslog(LOG_ERR, "adc_main: read size=%ld is not a multiple of "
//                         "sample size=%d, Ignoring\n",
//                (long)adc3_config.nbytes, sizeof(struct adc_msg_s));
//       }
//       else
//       {
//         syslog(LOG_INFO, "Sample:\n");
//         for (i = 0; i < nsamples; i++)
//         {
//           syslog(LOG_INFO, "%d: channel: %d value: %" PRId32 "\n",
//                  i, int_adc3_sample[i].am_channel, int_adc3_sample[i].am_data);
//         }
//       }
//     }

//     if (g_adcstate3.count && --g_adcstate3.count <= 0)
//     {
//       break;
//     }
//   }

//   close(adc3_config.fd);
//   return OK;

// /* Error exits */
// errout_with_dev:
//   close(adc3_config.fd);

// errout:
//   syslog(LOG_ERR, "[adc_main]: Terminating!\n");
//   fflush(stdout);
//   close(adc3_config.fd);
//   return adc3_config.errval;
// }
#endif

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

void int_adc1_data_convert(float *temp_buff, FAR const struct adc_msg_s* sample)
{
  struct adc_msg_s temp =sample[14];

  // float temp_buff[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_NSAMPLES];
  float ADC_SUP = 1.2 * 4095 / (temp.am_data);

  for (int i = 0; i < CONFIG_CUSTOM_APPS_INT_ADC1_GROUPSIZE; i++)
  {
    temp = sample[i];
    temp_buff[i] = (temp.am_data * 3.3 / 4095); // right now, using 1.2 as vrefint channel data... converting all the data to their respective voltages
    if (i == 4)
    {
      temp_buff[i] = (temp_buff[i] * (1100 + 931)) / 931; // this is for battery monitor (voltage data, no need for conversion)
    }
    else if (i == 9 || i == 10 || i == 11) // this one is for battery current, solar panel total current and raw current respectively
    {
      temp_buff[i] = ((temp_buff[i] - 1.65) / SENS_TMCS) * 1000;
    }
    else
    {
      // all current sensors use lmp8640 current sensor ...
      temp_buff[i] = (temp_buff[i] / (2 * RES_LMP8640 * GAIN_LMP8640)) * 1000;
    }
  }
}

/****************************************************************************
 * Name: int_adc3_data_convert
 *
 * Parameters:  *temp_buff -- float pointer to the data that is converted
 *
 * Details:
 *    only one channel data is converted by internal adc 3 i.e. 4V_I
 *    it uses LMP8640 current sensor, so same formula given above is used ...
 ****************************************************************************/

void int_adc3_data_convert(float *temp_buff_1, FAR const struct adc_msg_s* sample)
{
  struct adc_msg_s temp;
  for (int i = 0; i < CONFIG_CUSTOM_APPS_INT_ADC3_GROUPSIZE; i++)
  {
    temp = sample[i];
    temp_buff_1[i] = (float)temp.am_data * 3.3 / 4095;
    temp_buff_1[i] = (float)(temp_buff_1[i] / (2 * RES_LMP8640 * GAIN_LMP8640)) * 1000;
  }
}

int adc_daemon(int argc, FAR char *argv[])
{

  g_adc_daemon_started = true;
  int ret;
  int adc_instance = 0;
  int adc_afd;
  static struct adc_state_s g_adcstate1;
  struct int_adc_config_s adc1_config;
  struct adc_msg_s int_adc1_sample[CONFIG_CUSTOM_APPS_INT_ADC1_GROUPSIZE];
  float int_adc1_temp[CONFIG_CUSTOM_APPS_INT_ADC1_GROUPSIZE];

  static struct adc_state_s g_adcstate3;
  struct int_adc_config_s adc3_config;
  struct adc_msg_s int_adc3_sample[CONFIG_CUSTOM_APPS_INT_ADC3_GROUPSIZE];
  float int_adc3_temp[CONFIG_CUSTOM_APPS_INT_ADC3_GROUPSIZE];

  /* Check if we have initialized */
  if (!g_adcstate1.initialized)
  {
    adc_devpath(&g_adcstate1, CONFIG_CUSTOM_APPS_INT_ADC1_DEVPATH);
    g_adcstate1.initialized = true;
  }

  /* Check if we have initialized */
  if (!g_adcstate3.initialized)
  {
    adc_devpath(&g_adcstate3, CONFIG_CUSTOM_APPS_INT_ADC3_DEVPATH);
    g_adcstate3.initialized = true;
  }
  g_adcstate1.count = CONFIG_CUSTOM_APPS_INT_ADC1_NSAMPLES;
  adc1_config.readsize = CONFIG_CUSTOM_APPS_INT_ADC1_GROUPSIZE * sizeof(struct adc_msg_s);
  g_adcstate3.count = CONFIG_CUSTOM_APPS_INT_ADC3_NSAMPLES;
  adc3_config.readsize = CONFIG_CUSTOM_APPS_INT_ADC3_GROUPSIZE * sizeof(struct adc_msg_s);

  struct sat_int_adc_msg int_adc;

  adc_afd = orb_advertise_multi_queue_persist(ORB_ID(sat_int_adc_msg), &int_adc,
                                              0, sizeof(struct sat_int_adc_msg));
  if (adc_afd < 0)
  {
    syslog(LOG_ERR, "[adc_main] Internal ADC message advertise failed.\n");
  }

  for (;;)
  {
    syslog(LOG_INFO, "[adc_main]: g_adcstate1.count: %d\n", g_adcstate1.count);
    read_int_adc(&g_adcstate1, &adc1_config, int_adc1_sample, CONFIG_CUSTOM_APPS_INT_ADC1_NSAMPLES);
    int_adc1_data_convert(int_adc1_temp, int_adc1_sample);

    syslog(LOG_INFO, "[adc_main] g_adcstate3.count: %d\n", g_adcstate3.count);
    read_int_adc(&g_adcstate3, &adc3_config, int_adc3_sample, CONFIG_CUSTOM_APPS_INT_ADC3_NSAMPLES);
    int_adc3_data_convert(int_adc3_temp, int_adc3_sample);

    int_adc.C_batt = int_adc1_temp[9];
    int_adc.C_SolT = int_adc1_temp[10];
    int_adc.C_raw = int_adc1_temp[11];
    int_adc.C_unreg = int_adc1_temp[0];
    int_adc.C_3v3_main = int_adc1_temp[1];
    int_adc.C_3v3_com = int_adc1_temp[2];
    int_adc.C_5v = int_adc1_temp[3];
    int_adc.volt_batt = int_adc1_temp[4];
    int_adc.C_sp1 = int_adc1_temp[5];
    int_adc.C_3v3_2 = int_adc1_temp[6];
    int_adc.C_sp4 = int_adc1_temp[7];
    int_adc.C_sp5 = int_adc1_temp[8];
    int_adc.C_sp2 = int_adc1_temp[12];
    int_adc.C_sp3 = int_adc1_temp[13];
    int_adc.C_4v = int_adc3_temp[0];
    int_adc.timestamp = orb_absolute_time();

    if(OK != orb_publish(ORB_ID(sat_int_adc_msg), adc_afd, &int_adc))
    {
      syslog(LOG_ERR,"[adc_main] Sat int adc publish failed.\n");
    }
    usleep(500000);
  } // for(;;)

  ret = orb_unadvertise(adc_afd);
  if(ret < 0)
  {
    syslog(LOG_ERR,"[adc_main] int adc orb unadvertise failed.\n");
  }
  return 0;
}

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{ 
  int ret;

  syslog(LOG_INFO,"[adc_main] Stating task.\n");
  if(g_adc_daemon_started)
  {
    syslog(LOG_WARNING,"[adc_main] Task already started.\n");
    return EXIT_SUCCESS;
  }

  ret = task_create("adc_daemon", SCHED_PRIORITY_DEFAULT,
                    CONFIG_CUSTOM_APPS_INT_ADC_STACKSIZE, adc_daemon,
                    NULL);
  if(ret < 0)
  {
    int errcode = errno;
    syslog(LOG_ERR,"[adc_main] ERROR: failed to start adc_daemon.%d\n",
           errcode);
    return EXIT_FAILURE;
  }

  syslog(LOG_INFO,"[adc_main] adc_daemon started.\n");
  return EXIT_SUCCESS;
}