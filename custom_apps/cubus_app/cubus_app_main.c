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

// #include <nuttx/config.h>
// #include <stdio.h>

// #include <string.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/progmem.h>
#include <fcntl.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/
static void adc_devpath(FAR struct adc_state_s *adc, FAR const char *devpath);

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
satellite_health_s sat_health_rx_buf = {'\0'};
ext_adc_s ext_adc_data[EXT_ADC_MAX_CHANNELS];

const char file_name_flag[] = {"/flags.txt"};
const char file_name_sat_health[] = {"/sat_health.txt"};

const char file_name_epdm_msn[] = {"/epdm.txt"};
const char file_name_cam_msn[] = {"/cam.txt"};
const char file_name_test_msn[] = {"/test_msn.txt"};

#define FLAG_DATA_INT_ADDR    0x081C0000
#define FLAG_DATA_SECTOR_NUM  

CRITICAL_FLAGS critic_flags;

/****************************************************************************
 * Name: main
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
  Setup();
  check_flag_data();

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

  store_sat_health_data(&sat_health);

  retrieve_latest_sat_health_data(&sat_health_rx_buf);

  //TODO: after checking flags data are being written/read correctly, we'll enable satellite health things as well and have a basic complete work queue functions except UART 

  print_critical_flag_data(&critic_flags);
  critic_flags.ANT_DEP_STAT = DEPLOYED;
  store_flag_data(&critic_flags);
  print_critical_flag_data(&critic_flags);  
  return 0;

}

int store_sat_health_data(struct satellite_health_s *sat_health_data){
  struct file file_p;
  //TODO: discuss and figure out if we need to set limit to size of file and truncate contents once the file size limit is reached ... 
  if(open_file_flash(&file_p, MFM_MAIN_STRPATH, file_name_sat_health, O_RDWR | O_APPEND) >= 0){
    ssize_t bytes_written = file_write(&file_p, sat_health_data, sizeof(satellite_health_s));
    if  (bytes_written > 0)
    {
      syslog(LOG_INFO, "Satellite Health data write Successful.\nData Len: %d.\n", bytes_written);
      file_close(&file_p);
    } else {
      syslog(LOG_INFO, "Write Failure.\n");
    }
    file_syncfs(&file_p);
    file_close(&file_p);
  }else{
    syslog(LOG_ERR, "Error opening file to write satellite health data..\n");
  }
  file_close(&file_p);
}

void retrieve_latest_sat_health_data(satellite_health_s *sat_health_buf){
  struct stat st;
  struct file fptr;
  int fd = 0;
  fd = open_file_flash(&fptr, MFM_MAIN_STRPATH, file_name_sat_health, O_RDONLY);
  if(fd >= 0){
    int size_file = file_seek(&fptr, 0, SEEK_END);
    int off = file_seek(&fptr, size_file - 76, SEEK_SET);

    //TODO: discuss and figure out if we need to set limit to size of file and truncate contents once the file size limit is reached ... 
    printf("Size of file : %d \n Offset: %d \n  ",size_file, off);
    ssize_t bytes_read = file_read(&fptr, sat_health_buf, sizeof(satellite_health_s));
    if  (bytes_read > 0)
    {
      syslog(LOG_INFO, "Flash Read Successful.\nData Len: %d.\n", bytes_read);
      file_close(&fptr);
      print_satellite_health_data(sat_health_buf);
    } else {
      syslog(LOG_INFO, "Read Failure.\n");
    }
    file_syncfs(&fptr);
    file_close(&fptr);
  }else{
    syslog(LOG_ERR, "Error opening file to read satellite health data..\n");
  }
  file_close(&fptr);
}


void retrieve_sat_health_data(satellite_health_s sat_health_buf[], int times){
  struct stat st;
  struct file fptr;
  int fd = 0;
  fd = open_file_flash(&fptr, MFM_MAIN_STRPATH, file_name_sat_health, O_RDONLY);
  if(fd >= 0){
    int size_file = file_seek(&fptr, 0, SEEK_END);
    int off = file_seek(&fptr, size_file - 76*times, SEEK_SET);
    printf("Size of file : %d \n Offset: %d \n  ",size_file, off);
    for(int i=0;i<times;i++){
      //TODO: discuss and figure out if we need to set limit to size of file and truncate contents once the file size limit is reached ... 
      ssize_t bytes_read = file_read(&fptr, &sat_health_buf[i], sizeof(satellite_health_s));
      if  (bytes_read > 0) {
        syslog(LOG_INFO, "Flash Read Successful.\nData Len: %d.\n", bytes_read);
        file_close(&fptr);
        print_satellite_health_data(&sat_health_buf[i]);
      } else {
        syslog(LOG_INFO, "Read Failure.\n");
      }
    }
    file_syncfs(&fptr);
    file_close(&fptr);
  }else{
    syslog(LOG_ERR, "Error opening file to read satellite health data..\n");
  }
  file_close(&fptr);
}

int get_file_size(char *filepath, char *filename){
  struct file fp;
  int fd = open_file_flash(&fp, filepath, filename, O_RDONLY);
  if(fd < 0){
    return -1;
  }
  int file_size = file_seek(&fp, 0, SEEK_END);
  file_close(&fp);
  return file_size;
}

int clear_file(char *fpath, char *fname){
  struct file f;
  int fd = open_file_flash(&f, fpath, fname, O_RDWR);
  if(fd < 0){
    return -1;
  }
  int size = file_seek(&f, 0, SEEK_END);
  file_truncate(&f, size);
  file_close(&f);
  return 0;
}

int check_flag_data(){
  CRITICAL_FLAGS rd_flags_int;

  // uint8_t write_buffer[6];
  // uint8_t read_buffer[6];

  CRITICAL_FLAGS rd_flags_mfm = {0xff};
  uint8_t mfm_have_data = 0;
  ssize_t read_size_mfm = 0;
  struct file fp;

  // irqstate_t flags;
  // flags = enter_critical_section();
  int fd  = open("/dev/intflash", O_RDWR); 
  if(fd >= 0){  //internal flash file opened successfully
    syslog(LOG_INFO, "Printing Internal flash flag data.\n");
    up_progmem_read(FLAG_DATA_INT_ADDR, &rd_flags_int, sizeof(rd_flags_int));
    print_critical_flag_data(&rd_flags_int);
  }else{
    syslog(LOG_ERR, "Error opening internal flash atempt 1......\n ");
  }

  close(fd);
  //TODO: discuss and decide whether we want to append the flags data or if we want flags data to be stored only one on the same folders ... 
  //INFO: for now, system will overwrite the previous flag data if it needs to update it, it can be changed by changing file open mode and will need file seek to set cursor/pointer to read data
  int fd1 = open_file_flash(&fp, MFM_MAIN_STRPATH, file_name_flag, O_RDWR );
  if(fd1 >= 0){
    read_size_mfm = file_read(&fp, &rd_flags_mfm, sizeof(CRITICAL_FLAGS));
  }

  //TODO: add code for shared flash memory as well.. for now only main flash and internal flash data are read....
  if(rd_flags_int.ANT_DEP_STAT == 0xff){ //no data available in internal flash
    if(read_size_mfm == 0 || read_size_mfm != sizeof(CRITICAL_FLAGS)){  //no data in mfm or data is corrupted
      syslog(LOG_INFO, "Data not available in internal flash....\n Data corrupted or not available in main flash\n Read size from mfm %d \n", read_size_mfm);
      
      syslog(LOG_INFO, "Printing Internal flash flag data.\n");
      print_critical_flag_data(&rd_flags_int);

      syslog(LOG_INFO, "Initializing all the flags to default...\n");
      critic_flags.ANT_DEP_STAT = UNDEPLOYED;
      critic_flags.KILL_SWITCH_STAT = KILL_SW_OFF;
      critic_flags.OPER_MODE = NRML_MODE; //later on we check for battery voltage, now set default normal mode
      critic_flags.RSV_FLAG = RSV_NOT_RUNNING;
      critic_flags.UL_STATE = UL_NOT_RX;

      file_truncate(&fp, sizeof(CRITICAL_FLAGS));
      file_write(&fp, &critic_flags, sizeof(CRITICAL_FLAGS));
      file_close(&fp);

      fd = open("/dev/intflash", O_RDWR); //open internal flash to write    
      if(fd >= 0){
        up_progmem_eraseblock(22);
        up_progmem_write(0x081C0000, &critic_flags, sizeof(CRITICAL_FLAGS));
      }else{
        syslog(LOG_ERR, "Unable to open internal flash to write flag data ... \n");
      }
      close(fd);  //closing internal flash just after writing

    }else{  //data available in main flash and is equal to the expected buffer size
        syslog(LOG_INFO, "Flag data available in Main Flash Memory only.. Data size: %d\n", read_size_mfm);
        file_close(&fp);

        syslog(LOG_INFO, "Printing Internal flash flag data.\n");
        print_critical_flag_data(&rd_flags_int);

        syslog(LOG_INFO, "copying from main flash to internal flash ...\n");
        critic_flags.ANT_DEP_STAT = rd_flags_mfm.ANT_DEP_STAT;
        critic_flags.KILL_SWITCH_STAT = rd_flags_mfm.KILL_SWITCH_STAT;
        critic_flags.OPER_MODE = rd_flags_mfm.OPER_MODE;
        critic_flags.RSV_FLAG = rd_flags_mfm.RSV_FLAG;
        critic_flags.UL_STATE = rd_flags_mfm.UL_STATE;

        // write_buffer[0] = 0x00;
        // write_buffer[1] = 0x00;
        // write_buffer[2] = 0x00;
        // write_buffer[3] = 90;
        // write_buffer[4] = 0x00;
        
        // syslog(LOG_INFO, "Printing Main flash flag data.\n");
        // print_critical_flag_data(&critic_flags);

        fd  = open("/dev/intflash", O_RDWR);
        if(fd >= 0){
          up_progmem_eraseblock(22);
          up_progmem_write(0x081C0000, &critic_flags, sizeof(CRITICAL_FLAGS));
        }else{
          syslog(LOG_ERR, "Unable to open internal flash to write flags data\n");
        }
        close(fd);
    }
  }else{  //data available in internal flash memory
    if(read_size_mfm == 0 || read_size_mfm != sizeof(CRITICAL_FLAGS)){  //data not available in main flash
      syslog(LOG_INFO, "Data available in internal flash memory only...\n Copying from internal flash to main flash\n");
      critic_flags.ANT_DEP_STAT = rd_flags_int.ANT_DEP_STAT;
      critic_flags.KILL_SWITCH_STAT = rd_flags_int.KILL_SWITCH_STAT;
      critic_flags.OPER_MODE = rd_flags_int.OPER_MODE;
      critic_flags.RSV_FLAG = rd_flags_int.RSV_FLAG;
      critic_flags.UL_STATE = rd_flags_int.UL_STATE;

      syslog(LOG_INFO, "Printing Internal flash flag data.\n");
      print_critical_flag_data(&rd_flags_int);
      // close(fd);  //closing internal flash 
      file_truncate(&fp, sizeof(CRITICAL_FLAGS));
      file_write(&fp, &critic_flags, sizeof(CRITICAL_FLAGS));
      file_close(&fp);
    }else{  //data available in both flash memories...
      //TODO: add sfm as well and check for data discrepancy from all three flash memories and go for data from majority ones
      //INFO: for now, data from internal flash is trusted... data from mfm is read just for debugging purposes (without three copies checking for data discrepancy makes no sense)
      syslog(LOG_INFO, "data available in both main flash and internal flash memories\n ");

      syslog(LOG_INFO, "Printing Main flash flag data....\n");
      print_critical_flag_data(&rd_flags_mfm);

      syslog(LOG_INFO, "Printing Internal flash flag data.\n");
      print_critical_flag_data(&rd_flags_int);

      int data_check = 0;
      critic_flags.ANT_DEP_STAT = rd_flags_int.ANT_DEP_STAT;
      critic_flags.KILL_SWITCH_STAT = rd_flags_int.KILL_SWITCH_STAT;
      critic_flags.OPER_MODE = rd_flags_int.OPER_MODE;
      critic_flags.RSV_FLAG = rd_flags_int.RSV_FLAG;
      critic_flags.UL_STATE = rd_flags_int.UL_STATE;
      // close(fd);
      file_truncate(&fp, sizeof(CRITICAL_FLAGS));
      file_write(&fp, &critic_flags, sizeof(CRITICAL_FLAGS));
      file_close(&fp);
    }
  }
  file_close(&fp);
  syslog(LOG_INFO, "Flags data check and write complete.... \n");
  // leave_critical_section(flags);
}


int store_flag_data(CRITICAL_FLAGS *flag_data){
  struct file fp;
  int bwr;
  int fd  = open("/dev/intflash",O_RDWR); 
  if(fd >= 0){  //internal flash file opened successfully
    up_progmem_eraseblock(22);
    up_progmem_write(FLAG_DATA_INT_ADDR, flag_data, sizeof(CRITICAL_FLAGS));
  }else{
    syslog(LOG_ERR, "Error opening internal flash to store new flag data ... \n ");
  }
  close(fd);
  //TODO: USE SFM AS WELL, FOR NOW ONLY MFM IS USED
  int fd1 = open_file_flash(&fp, MFM_MAIN_STRPATH, file_name_flag, O_RDWR);

  if(fd1 >= 0){

    file_truncate(&fp, sizeof(CRITICAL_FLAGS)); //clearing out any previous data 
    bwr = file_write(&fp, flag_data, sizeof(CRITICAL_FLAGS));
    if(bwr == 0 || bwr != sizeof(CRITICAL_FLAGS)){
      syslog(LOG_ERR, "Error in writing flag data to MFM\n Will try once again without verifying...\n");

      file_truncate(&fp, sizeof(CRITICAL_FLAGS));
      bwr = file_write(&fp, flag_data, sizeof(CRITICAL_FLAGS));
      syslog(LOG_INFO, "Size of flag data written to MFM on second attempt: %d",bwr);
    }
  }else{
    syslog(LOG_ERR, "Unable to open %s%s for writing critical flash data\n",MFM_MAIN_STRPATH, file_name_flag);
  }
  file_close(&fp);
}

// /*
// */
void Setup(){
  int fd = 0;
  struct file flp1, flp2, flp3, flp4;
  fd = open_file_flash(&flp1, MFM_MAIN_STRPATH, file_name_sat_health, O_CREAT);
  if(fd < 0){
    syslog(LOG_ERR, "Could not create file ... \n");
  }
  file_close(&flp1);
  fd = open_file_flash(&flp2, MFM_MAIN_STRPATH, file_name_flag, O_CREAT);
  if(fd < 0){
    syslog(LOG_ERR, "Could not create flags data file ... \n");
  }
  file_close(&flp2);

  fd = open_file_flash(&flp3, MFM_MSN_STRPATH, file_name_cam_msn, O_CREAT);
  if(fd < 0){
    syslog(LOG_ERR, "could not create cam msn file ...\n");
  }
  file_close(&flp3);

  fd = open_file_flash(&flp4, MFM_MSN_STRPATH, file_name_epdm_msn, O_CREAT);
  if(fd < 0){
    syslog(LOG_ERR, "Could not create epddm msn file\n");
  }
  file_close(&flp4);

  //TODO: check flags thing and put the check flags data into this function for now. Later, after uORB or uMSG is ready, we'll have a setup application and we wont need to put that in this function. Architecture will be completely different. 

}


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
#endif  //CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

int open_file_flash(struct file *file_pointer, char *flash_strpath, char *filename, int open_mode){

  const char file_name[] = {'\0'};
  // memcpy(file_name, filename, sizeof(filename));
  char path[65];
  sprintf(path, "%s%s", flash_strpath, filename);
  int fd = file_open(file_pointer, path, open_mode);
  if(fd < 0){
     syslog(LOG_ERR, "Error opening file: %s\n",path);
     return fd;
  }else{
    syslog(LOG_INFO, "Opened file: %s ...\n",path);
  }
  return fd;
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

/*
*/
void print_critical_flag_data(CRITICAL_FLAGS *flags){

  printf(" ********************************************\r\n");
  printf(" |   Antenna Deployment Status \t %d \t|\r\n",flags->ANT_DEP_STAT);
  printf(" |   Kill Switch Status        \t %d \t|\r\n",flags->KILL_SWITCH_STAT);
  printf(" |   Operation Mode            \t %d \t|\r\n",flags->OPER_MODE);
  printf(" |   Reservation Table Flag    \t %d \t|\r\n",flags->RSV_FLAG);
  printf(" |   Command uplink status     \t %d \t|\r\n",flags->UL_STATE);
  printf(" ********************************************\r\n");
}


