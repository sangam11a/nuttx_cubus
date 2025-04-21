// /****************************************************************************
//  * apps/custom_apps/cubus_app/common_functions.c
//  *
//  * Licensed to the Apache Software Foundation (ASF) under one or more
//  * contributor license agreements.  See the NOTICE file distributed with
//  * this work for additional information regarding copyright ownership.  The
//  * ASF licenses this file to you under the Apache License, Version 2.0 (the
//  * "License"); you may not use this file except in compliance with the
//  * License.  You may obtain a copy of the License at
//  *
//  *   http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
//  * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
//  * License for the specific language governing permissions and limitations
//  * under the License.
//  *
//  ****************************************************************************/

#include "common_functions.h"
// pthread_mutex_t flash_mutex =  pthread_mutex_INITIALIZER;

void mission_data(char *filename, uint8_t *data, uint16_t size1)
{
  struct file file_p;
  // TODO: discuss and figure out if we need to set limit to size of file and truncate contents once the file size limit is reached ...
  if (open_file_flash(&file_p, "/mnt/fs/mfm", filename, O_CREAT | O_RDWR | O_APPEND) >= 0)
  {
    ssize_t bytes_written = file_write(&file_p, data, size1);
    // writer_mq_edited(sat_health_data);
    if (bytes_written > 0)
    {
      syslog(LOG_INFO, "Satellite Health data write Successful.\nData Len: %d.\n", bytes_written);
      file_close(&file_p);
    }
    else
    {
      syslog(LOG_INFO, "Write Failure.\n");
    }
    file_syncfs(&file_p);
    file_close(&file_p);
  }
  else
  {
    syslog(LOG_ERR, "Error opening file to write satellite health data..\n");
  }
  file_close(&file_p);
  // TODO delter this later
}


int clear_int_flag(){
  CRITICAL_FLAGS c={'\0'};
  struct file fp;
  int fd;
  // pthread_mutex_lock(&flash_mutex); // Lock the mutex

  fd = open("/dev/intflash", O_WRONLY);
  up_progmem_eraseblock(22);
  up_progmem_write(FLAG_DATA_INT_ADDR, c, sizeof(CRITICAL_FLAGS));

  close(fd);
  // fd = file_open(&fp, "/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_TRUNC);
  // file_close(&fp);
  // pthread_mutex_unlock(&flash_mutex); // Lock the mutex

}

int store_flag_data(CRITICAL_FLAGS *flag_data)
{
  struct file fp;
  int bwr;

  printf("\n**************SToring flag data*********\n");
  print_critical_flag_data(flag_data);

  int fd = open("/dev/intflash", O_RDWR);
  if (fd >= 0)
  {
    up_progmem_eraseblock(22);
    up_progmem_write(FLAG_DATA_INT_ADDR, flag_data, sizeof(CRITICAL_FLAGS));
    close(fd);
  }
  else
  {
    syslog(LOG_ERR, "Error opening internal flash to store new flag data ... \n");
    return -1;
  }
  
  int fd1 = open_file_flash(&fp, MFM_MAIN_STRPATH, file_name_flag, O_RDWR);
  if (fd1 >= 0)
  {
    file_truncate(&fp, sizeof(CRITICAL_FLAGS));
    bwr = file_write(&fp, flag_data, sizeof(CRITICAL_FLAGS));
    if (bwr != sizeof(CRITICAL_FLAGS))
    {
      syslog(LOG_ERR, "Error in writing flag data to MFM\n");
    }
    file_close(&fp);
  }
  else
  {
    syslog(LOG_ERR, "Unable to open %s%s for writing critical flash data\n", MFM_MAIN_STRPATH, file_name_flag);
    return -1;
  }
  
  syslog(LOG_DEBUG, "\n-----Storing data to flash-----\n");
  return 0;
}

int check_flag_data(CRITICAL_FLAGS *flags)
{
  CRITICAL_FLAGS rd_flags_int = {0};
  CRITICAL_FLAGS rd_flags_mfm = {0xff};
  ssize_t read_size_mfm = 0;
  struct file fp;

  int fd = open("/dev/intflash", O_RDWR);
  if (fd >= 0)
  {
    syslog(LOG_INFO, "Printing Internal flash flag data.\n");
    up_progmem_read(FLAG_DATA_INT_ADDR, &rd_flags_int, sizeof(rd_flags_int));
    print_critical_flag_data(&rd_flags_int);
    close(fd);
  }
  else
  {
    syslog(LOG_ERR, "Error opening internal flash\n");
    return -1;
  }

  int fd1 = file_open(&fp, "/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_RDWR);
  if (fd1 >= 0)
  {
    read_size_mfm = file_read(&fp, &rd_flags_mfm, sizeof(CRITICAL_FLAGS));
    file_close(&fp);
  }

  if (rd_flags_int.ANT_DEP_STAT != DEPLOYED)
  {
    if (read_size_mfm != sizeof(CRITICAL_FLAGS))
    {
      syslog(LOG_INFO, "Data not available or corrupted in main flash\n");

      critic_flags.ANT_DEP_STAT = UNDEPLOYED;
      critic_flags.KILL_SWITCH_STAT = KILL_SW_OFF;
      critic_flags.OPER_MODE = NRML_MODE;
      critic_flags.RSV_FLAG = RSV_NOT_RUNNING;
      critic_flags.UL_STATE = UL_NOT_RX;

      store_flag_data(&critic_flags);
    }
    else
    {
      syslog(LOG_INFO, "Flag data available in Main Flash Memory\n");

      critic_flags = rd_flags_mfm;
      store_flag_data(&critic_flags);
    }
  }
  else
  {
    if (read_size_mfm != sizeof(CRITICAL_FLAGS))
    {
      syslog(LOG_INFO, "Data available in internal flash only\n");

      critic_flags = rd_flags_int;
      store_flag_data(&critic_flags);
    }
    else
    {
      syslog(LOG_INFO, "Data available in both main flash and internal flash memories\n");

      critic_flags = rd_flags_int;
    }
  }

  *flags = critic_flags;
  return 0;
}


void save_critics_flags(const CRITICAL_FLAGS *flags)
{
  int fd = open("/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_WRONLY | O_CREAT);
  if (fd < 0)
  {
    perror("Failed to open flags.txt for writing");
    return;
  }
  write(fd, flags, sizeof(CRITICAL_FLAGS));
  close(fd);
}

int load_critics_flags(CRITICAL_FLAGS *flags)
{
  memset(flags, '\0', sizeof(CRITICAL_FLAGS));
  int fd = open("/dev/intflash", O_RDONLY); /// mnt/fs/mfm/mtd_mainstorage/
  if (fd < 0)
  {
    perror("Failed to open /dev/intflash for reading");
    return -1;
  }
  ssize_t bytesRead = read(fd, flags, sizeof(CRITICAL_FLAGS));
  close(fd);
  if (bytesRead != sizeof(CRITICAL_FLAGS))
  {
    perror("Failed to read complete flags data");
    return -1;
  }
  return 0;
}



// void Setup()
// {
//   int fd = 0;
//   struct file flp1, flp2, flp3, flp4;
//   fd = open_file_flash(&flp1, MFM_MAIN_STRPATH, file_name_sat_health, O_CREAT);
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "Could not create file named sat_health... \n");
//   }
//   file_close(&flp1);

//   /*delete this later
//   */
//      fd = open_file_flash(&flp1, MFM_MAIN_STRPATH, file_name_sat_health, O_TRUNC);
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "Could not create file named sat_health... \n");
//   }
//   file_close(&flp1);
//   close(fd);
//  return 0;
// /*
//   */
//   fd = open_file_flash(&flp2, MFM_MAIN_STRPATH, file_name_flag, O_CREAT);
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "Could not create flags.txt data file ... \n");
//   }
//   file_close(&flp2);

//   fd = open_file_flash(&flp3, MFM_MSN_STRPATH, file_name_cam_msn, O_CREAT);
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "could not create cam.txt msn file ...\n");
//   }
//   file_close(&flp3);

//   fd = open_file_flash(&flp4, MFM_MSN_STRPATH, file_name_epdm_msn, O_CREAT);
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "Could not create epdm.txt msn file\n");
//   }
//   file_close(&flp4);

//   syslog(LOG_INFO, "Checking initial flag data...\n");
//   check_flag_data();
//   print_critical_flag_data(&critic_flags);
// }

void print_critical_flag_data(CRITICAL_FLAGS *flags)
{
  CRITICAL_FLAGS rd_flags_int = {0xff};

  int fd = open("/dev/intflash", O_RDONLY);
  if (fd >= 0)
  { // internal flash file opened successfully
    syslog(LOG_INFO, "Printing Internal flash flag data.\n");
    up_progmem_read(FLAG_DATA_INT_ADDR, &rd_flags_int, sizeof(rd_flags_int));
    // print_critical_flag_data(&rd_flags_int);
  }

  printf(" ********************************************\r\n");
  printf(" |   Antenna Deployment Status \t %d \t|\r\n", flags->ANT_DEP_STAT);
  printf(" |   Kill Switch Status        \t %d \t|\r\n", flags->KILL_SWITCH_STAT);
  printf(" |   Operation Mode            \t %d \t|\r\n", flags->OPER_MODE);
  printf(" |   Reservation Table Flag    \t %d \t|\r\n", flags->RSV_FLAG);
  printf(" |   Command uplink status     \t %d \t|\r\n", flags->UL_STATE);
  printf(" |   Reset counter     \t %d \t|\r\n", flags->RST_COUNT);

  printf(" ********************************************\r\n");
  close(fd);
}

struct KILL_SW{
 time_t timestamps[3];
    int count;
};
void add_command_timestamp(struct KILL_SW *kill_sw, time_t timestamp) {
    if (kill_sw->count < 3) {
        kill_sw->timestamps[kill_sw->count++] = timestamp;
    } else {
        for (int i = 1; i < 3; ++i) {
            kill_sw->timestamps[i - 1] = kill_sw->timestamps[i];
        }
        kill_sw->timestamps[3 - 1] = timestamp;
    }
}

int check_command_times(struct KILL_SW *kill_sw) {
    if (kill_sw->count < 3) {
        return 0;
    }
    
    time_t first_time = kill_sw->timestamps[0];
    time_t last_time = kill_sw->timestamps[3 - 1];
    
    return (difftime(last_time, first_time) <= 600);
}

int receive_command(struct KILL_SW *kill_sw) {
    time_t current_time = time(NULL);
    add_command_timestamp(kill_sw, current_time);

    if (check_command_times(kill_sw)) {
        // save_data();
        // kill_sw.
        // Reset kill_sw after saving data
        kill_sw->count = 0;
        return 0;
    }
    return 1;
}
