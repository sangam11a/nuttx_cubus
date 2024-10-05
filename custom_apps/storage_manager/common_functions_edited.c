/****************************************************************************
 * apps/custom_apps/cubus_app/common_functions.c
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

// #include "common_functions.h"
#include "common_functions_edited.h"

#include <mqueue.h>

CRITICAL_FLAGS critic_flags;

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

void store_sat_health_data(satellite_health_s *sat_health_data)
{
  struct file file_p;
  // TODO: discuss and figure out if we need to set limit to size of file and truncate contents once the file size limit is reached ...
  if (open_file_flash(&file_p, SFM_MAIN_STRPATH, file_name_sat_health, O_RDWR | O_APPEND) >= 0)
  {
    ssize_t bytes_written = file_write(&file_p, sat_health_data, sizeof(satellite_health_s));
    writer_mq_edited(sat_health_data);
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
}

void retrieve_latest_sat_health_data(satellite_health_s *sat_health_buf)
{
  printf("**************************\n***********************************************************\n Reading MFM data\n");
  struct stat st;
  struct file fptr;
  int fd = 0;
  
  fd = open_file_flash(&fptr, SFM_MAIN_STRPATH, file_name_sat_health, O_RDONLY);
  if (fd >= 0)
  {
    int size_file = file_seek(&fptr, 0, SEEK_END);
    int off = file_seek(&fptr, size_file - 112, SEEK_SET);

    // TODO: discuss and figure out if we need to set limit to size of file and truncate contents once the file size limit is reached ...
    printf("Size of file : %d \n Offset: %d \n  ", size_file, off);
    ssize_t bytes_read = file_read(&fptr, sat_health_buf, sizeof(satellite_health_s));
    if (bytes_read > 0)
    {
      syslog(LOG_INFO, "Flash Read Successful.\nData Len: %d.\n", bytes_read);
      file_close(&fptr);
      print_satellite_health_data(sat_health_buf);
    }
    else
    {
      syslog(LOG_INFO, "Read Failure.\n");
    }
    file_syncfs(&fptr);
    file_close(&fptr);
  }
  else
  {
    syslog(LOG_ERR, "Error opening file to read satellite health data..\n");
  }
  file_close(&fptr);
 
  printf("**************************\n***********************************************************\n Reading MFM data completed\n");
}

void retrieve_sat_health_data(satellite_health_s sat_health_buf[], int times)
{
  struct stat st;
  struct file fptr;
  int fd = 0;
  fd = open_file_flash(&fptr, MFM_MAIN_STRPATH, file_name_sat_health, O_RDONLY);
  if (fd >= 0)
  {
    int size_file = file_seek(&fptr, 0, SEEK_END);
    int off = file_seek(&fptr, size_file - 112 * times, SEEK_SET);
    printf("Size of file : %d \n Offset: %d \n  ", size_file, off);
    for (int i = 0; i < times; i++)
    {
      // TODO: discuss and figure out if we need to set limit to size of file and truncate contents once the file size limit is reached ...
      ssize_t bytes_read = file_read(&fptr, &sat_health_buf[i], sizeof(satellite_health_s));
      if (bytes_read > 0)
      {
        syslog(LOG_INFO, "Flash Read Successful.\nData Len: %d.\n", bytes_read);
        file_close(&fptr);
        print_satellite_health_data(&sat_health_buf[i]);
      }
      else
      {
        syslog(LOG_INFO, "Read Failure.\n");
      }
    }
    file_syncfs(&fptr);
    file_close(&fptr);
  }
  else
  {
    syslog(LOG_ERR, "Error opening file to read satellite health data..\n");
  }
  file_close(&fptr);
}

int check_flag_data()
{

  CRITICAL_FLAGS rd_flags_int;

  // uint8_t write_buffer[6];
  // uint8_t read_buffer[6];

  CRITICAL_FLAGS rd_flags_mfm = {0xff};
  uint8_t mfm_have_data = 0;
  ssize_t read_size_mfm = 0;
  struct file fp;

  // irqstate_t flags;
  // flags = enter_critical_section();
  int fd = open("/dev/intflash", O_RDWR);
  if (fd >= 0)
  { // internal flash file opened successfully
    syslog(LOG_INFO, "Printing Internal flash flag data.\n");
    up_progmem_read(FLAG_DATA_INT_ADDR, &rd_flags_int, sizeof(rd_flags_int));
    print_critical_flag_data(&rd_flags_int);
  }
  else
  {
    syslog(LOG_ERR, "Error opening internal flash atempt 1......\n ");
  }

  close(fd);
  // TODO: discuss and decide whether we want to append the flags data or if we want flags data to be stored only one on the same folders ...
  // INFO: for now, system will overwrite the previous flag data if it needs to update it, it can be changed by changing file open mode and will need file seek to set cursor/pointer to read data
  int fd1 = open_file_flash(&fp, MFM_MAIN_STRPATH, file_name_flag, O_RDWR);
  if (fd1 >= 0)
  {
    read_size_mfm = file_read(&fp, &rd_flags_mfm, sizeof(CRITICAL_FLAGS));
  }

  // TODO: add code for shared flash memory as well.. for now only main flash and internal flash data are read....
  if (rd_flags_int.ANT_DEP_STAT == 0xff)
  { // no data available in internal flash
    if (read_size_mfm == 0 || read_size_mfm != sizeof(CRITICAL_FLAGS))
    { // no data in mfm or data is corrupted
      syslog(LOG_INFO, "Data not available in internal flash....\n Data corrupted or not available in main flash\n Read size from mfm %d \n", read_size_mfm);

      syslog(LOG_INFO, "Printing Internal flash flag data.\n");
      print_critical_flag_data(&rd_flags_int);

      syslog(LOG_INFO, "Initializing all the flags to default...\n");
      critic_flags.ANT_DEP_STAT = UNDEPLOYED;
      critic_flags.KILL_SWITCH_STAT = KILL_SW_OFF;
      critic_flags.OPER_MODE = NRML_MODE; // later on we check for battery voltage, now set default normal mode
      critic_flags.RSV_FLAG = RSV_NOT_RUNNING;
      critic_flags.UL_STATE = UL_NOT_RX;

      file_truncate(&fp, sizeof(CRITICAL_FLAGS));
      file_write(&fp, &critic_flags, sizeof(CRITICAL_FLAGS));
      file_close(&fp);

      fd = open("/dev/intflash", O_RDWR); // open internal flash to write
      if (fd >= 0)
      {
        up_progmem_eraseblock(22);
        up_progmem_write(0x081C0000, &critic_flags, sizeof(CRITICAL_FLAGS));
      }
      else
      {
        syslog(LOG_ERR, "Unable to open internal flash to write flag data ... \n");
      }
      close(fd); // closing internal flash just after writing
    }
    else
    { // data available in main flash and is equal to the expected buffer size
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

      fd = open("/dev/intflash", O_RDWR);
      if (fd >= 0)
      {
        up_progmem_eraseblock(22);
        up_progmem_write(0x081C0000, &critic_flags, sizeof(CRITICAL_FLAGS));
      }
      else
      {
        syslog(LOG_ERR, "Unable to open internal flash to write flags data\n");
      }
      close(fd);
    }
  }
  else
  { // data available in internal flash memory
    if (read_size_mfm == 0 || read_size_mfm != sizeof(CRITICAL_FLAGS))
    { // data not available in main flash
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
    }
    else
    { // data available in both flash memories...
      // TODO: add sfm as well and check for data discrepancy from all three flash memories and go for data from majority ones
      // INFO: for now, data from internal flash is trusted... data from mfm is read just for debugging purposes (without three copies checking for data discrepancy makes no sense)
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
  return 0;
}

int store_flag_data(CRITICAL_FLAGS *flag_data)
{
  struct file fp;
  int bwr;
  int fd = open("/dev/intflash", O_RDWR);
  if (fd >= 0)
  { // internal flash file opened successfully
    up_progmem_eraseblock(22);
    up_progmem_write(FLAG_DATA_INT_ADDR, flag_data, sizeof(CRITICAL_FLAGS));
  }
  else
  {
    syslog(LOG_ERR, "Error opening internal flash to store new flag data ... \n ");
  }
  close(fd);
  // TODO: USE SFM AS WELL, FOR NOW ONLY MFM IS USED
  int fd1 = open_file_flash(&fp, MFM_MAIN_STRPATH, file_name_flag, O_RDWR);

  if (fd1 >= 0)
  {

    file_truncate(&fp, sizeof(CRITICAL_FLAGS)); // clearing out any previous data
    bwr = file_write(&fp, flag_data, sizeof(CRITICAL_FLAGS));
    if (bwr == 0 || bwr != sizeof(CRITICAL_FLAGS))
    {
      syslog(LOG_ERR, "Error in writing flag data to MFM\n Will try once again without verifying...\n");

      file_truncate(&fp, sizeof(CRITICAL_FLAGS));
      bwr = file_write(&fp, flag_data, sizeof(CRITICAL_FLAGS));
      syslog(LOG_INFO, "Size of flag data written to MFM on second attempt: %d", bwr);
    }
  }
  else
  {
    syslog(LOG_ERR, "Unable to open %s%s for writing critical flash data\n", MFM_MAIN_STRPATH, file_name_flag);
  }
  file_close(&fp);
  return 0;
}

/*
 */
void Setup()
{
  int fd = 0;
  struct file flp1, flp2, flp3, flp4;
  fd = open_file_flash(&flp1, MFM_MAIN_STRPATH, file_name_sat_health, O_CREAT);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Could not create file named sat_health... \n");
  }
  file_close(&flp1);

  /*delete this later
  */
     fd = open_file_flash(&flp1, MFM_MAIN_STRPATH, file_name_sat_health, O_TRUNC);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Could not create file named sat_health... \n");
  }
  file_close(&flp1);
  close(fd);
 return 0;
/*
  */
  fd = open_file_flash(&flp2, MFM_MAIN_STRPATH, file_name_flag, O_CREAT);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Could not create flags.txt data file ... \n");
  }
  file_close(&flp2);

  fd = open_file_flash(&flp3, MFM_MSN_STRPATH, file_name_cam_msn, O_CREAT);
  if (fd < 0)
  {
    syslog(LOG_ERR, "could not create cam.txt msn file ...\n");
  }
  file_close(&flp3);

  fd = open_file_flash(&flp4, MFM_MSN_STRPATH, file_name_epdm_msn, O_CREAT);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Could not create epdm.txt msn file\n");
  }
  file_close(&flp4);

  syslog(LOG_INFO, "Checking initial flag data...\n");
  check_flag_data();
  print_critical_flag_data(&critic_flags);
}

/*
 */
void print_satellite_health_data(satellite_health_s *sat_health)
{
  printf(" *******************************************\r\n");
  printf(" |   X axis acceleration    \t %f \t|\r\n", sat_health->accl_x);
  printf(" |   Y axis acceleration    \t %f \t|\r\n", sat_health->accl_y);
  printf(" |   Z axis acceleration    \t %f \t|\r\n", sat_health->accl_z);

  printf(" |   X axis Gyro data       \t %f \t|\r\n", sat_health->gyro_x);
  printf(" |   Y axis Gyro data       \t %f \t|\r\n", sat_health->gyro_y);
  printf(" |   Z axis gyro data       \t %f \t|\r\n", sat_health->gyro_z);

  printf(" |   X axis magnetic field  \t %f \t|\r\n", sat_health->mag_x);
  printf(" |   Y axis magnetic field  \t %f \t|\r\n", sat_health->mag_y);
  printf(" |   Z axis magnetic field  \t %f \t|\r\n", sat_health->mag_z);

  printf(" |   Solar Panel 1 Voltage: \t %d \t|\r\n", sat_health->sol_p1_v);
  printf(" |   Solar Panel 2 Voltage: \t %d \t|\r\n", sat_health->sol_p2_v);
  printf(" |   Solar Panel 3 Voltage: \t %d \t|\r\n", sat_health->sol_p3_v);
  printf(" |   Solar Panel 4 Voltage: \t %d \t|\r\n", sat_health->sol_p4_v);
  printf(" |   Solar Panel 5 Voltage: \t %d \t|\r\n", sat_health->sol_p5_v);
  printf(" |   Solar Panel T Voltage: \t %d \t|\r\n", sat_health->sol_t_v);
  printf(" |--------------------------------------|\r\n");
  printf(" |   Solar Panel 1 Current: \t %d \t|\r\n", sat_health->sol_p1_c);
  printf(" |   Solar Panel 2 Current: \t %d \t|\r\n", sat_health->sol_p2_c);
  printf(" |   Solar Panel 3 Current: \t %d \t|\r\n", sat_health->sol_p3_c);
  printf(" |   Solar Panel 4 Current: \t %d \t|\r\n", sat_health->sol_p4_c);
  printf(" |   Solar Panel 5 Current: \t %d \t|\r\n", sat_health->sol_p5_c);
  printf(" |   Solar Panel T Current: \t %d \t|\r\n", sat_health->sol_t_c);
  printf(" |--------------------------------------|\r\n");
  printf(" |   Unreg Line Current:    \t %d \t|\r\n", sat_health->unreg_c);
  printf(" |   Main 3v3 Current:      \t %d \t|\r\n", sat_health->v3_main_c);
  printf(" |   COM 3v3 Current:       \t %d \t|\r\n", sat_health->v3_com_c);
  printf(" |   5 Volts line Current:  \t %d \t|\r\n", sat_health->v5_c);
  printf(" |   3v3 2 line Current:    \t %d \t|\r\n", sat_health->v3_2_c);
  printf(" |--------------------------------------|\r\n");
  printf(" |   Raw Current:           \t %d \t|\r\n", sat_health->raw_c);
  printf(" |   Raw Voltage:           \t %d \t|\r\n", sat_health->raw_v);
  printf(" |--------------------------------------|\r\n");
  printf(" |   Battery Total Voltage: \t %d \t|\r\n", sat_health->batt_volt);
  printf(" |   Battery Total Current: \t %d \t|\r\n", sat_health->batt_c);
  printf(" |   Battery Temperature:   \t %d \t|\r\n", sat_health->temp_batt);
  printf(" *********************************************\r\n");
}

/*
 */
void print_critical_flag_data(CRITICAL_FLAGS *flags)
{

  printf(" ********************************************\r\n");
  printf(" |   Antenna Deployment Status \t %d \t|\r\n", flags->ANT_DEP_STAT);
  printf(" |   Kill Switch Status        \t %d \t|\r\n", flags->KILL_SWITCH_STAT);
  printf(" |   Operation Mode            \t %d \t|\r\n", flags->OPER_MODE);
  printf(" |   Reservation Table Flag    \t %d \t|\r\n", flags->RSV_FLAG);
  printf(" |   Command uplink status     \t %d \t|\r\n", flags->UL_STATE);
  printf(" ********************************************\r\n");
}

/*
 */
int gpio_write1(uint32_t pin, uint8_t mode)
{

  gpio_config_s gpio_numval;
  int fd = open(ETX_LED_DRIVER_PATH, O_WRONLY);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Error opening %s for GPIO WRITE...", ETX_LED_DRIVER_PATH);
    close(fd);
    return -1;
  }
  gpio_numval.gpio_num = pin;
  gpio_numval.gpio_val = mode;
  if (gpio_numval.gpio_val > 1 || gpio_numval.gpio_num < 0)
  {
    syslog(LOG_ERR, "Undefined GPIO pin or set mode selected...\n");
    return -2;
  }
  int ret = write(fd, (const void *)&gpio_numval, sizeof(gpio_config_s));
  close(fd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Unable to write to gpio pin...\n");
  }
  return ret;
}

int gpio_read(uint32_t pin)
{
  gpio_config_s gpio_read;
  int fd = open(ETX_LED_DRIVER_PATH, O_WRONLY);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Error opening %s for GPIO WRITE...", ETX_LED_DRIVER_PATH);
    close(fd);
    return -1;
  }
  gpio_read.gpio_num = pin;
  if (gpio_read.gpio_num < 0)
  {
    syslog(LOG_ERR, "Undefined GPIO pin or set mode selected...\n");
    return -2;
  }
  int ret = read(fd, (const void *)&gpio_read, sizeof(gpio_config_s));
  close(fd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Unable to write to gpio pin...\n");
    return -3;
  }
  return gpio_read.gpio_val;
}