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
pthread_mutex_t flash_mutex = PTHREAD_MUTEX_INITIALIZER;

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

int clear_int_flag()
{
  CRITICAL_FLAGS c = {255};
  struct file fp;
  int fd;
  pthread_mutex_lock(&flash_mutex); // Lock the mutex
  //  flags = enter_critical_section();
  // irqstate_t flags = enter_critical_section();

  fd = open("/dev/intflash", O_RDWR);
  up_progmem_eraseblock(22);
  up_progmem_write(FLAG_DATA_INT_ADDR, c, sizeof(CRITICAL_FLAGS));

  close(fd);
  // leave_critical_section(flags);
  fd = file_open(&fp, "/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_TRUNC);
  file_close(&fp);
  pthread_mutex_unlock(&flash_mutex); // Lock the mutex
}

// int store_flag_data(CRITICAL_FLAGS *flag_data)
// {
//     struct file fp;
//     int bwr, ret = 0;

//     printf("\n************** Storing flag data *********\n");
//     print_critical_flag_data(flag_data);
//     toggle_wdg();

//     // Enter critical section
//     // irqstate_t flags = enter_critical_section();

//     // int fd = open("/dev/intflash", O_RDWR);
//     // if (fd >= 0)
//     // {
//     //     if (up_progmem_eraseblock(22) < 0)
//     //     {
//     //         syslog(LOG_ERR, "Error erasing flash block\n");
//     //         ret = -1;
//     //     }
//     //     else{
//     //       if (up_progmem_write(FLAG_DATA_INT_ADDR, flag_data, sizeof(CRITICAL_FLAGS)) < 0)
//     //       {
//     //           syslog(LOG_ERR, "Error writing to flash memory\n");
//     //           ret = -1;
//     //       }
//     //       else{
//     //         printf("Written to internal flash success\n");
//     //       }
//     //       close(fd);
//     //     }
//     //     // else
//     // }
//     // else
//     // {
//     //     syslog(LOG_ERR, "Error opening internal flash to store new flag data\n");
//     //     ret = -1;
//     // }

//     // // Exit critical section
//     // leave_critical_section(flags);

//     if (ret == 0)
//     {
//         int fd1 = open_file_flash(&fp, MFM_MAIN_STRPATH, file_name_flag, O_RDWR);
//         if (fd1 >= 0)
//         {
//             if (file_truncate(&fp, sizeof(CRITICAL_FLAGS)) < 0)
//             {
//                 syslog(LOG_ERR, "Error truncating file\n");
//                 ret = -1;
//             }
//             else
//             {
//                 bwr = file_write(&fp, flag_data, sizeof(CRITICAL_FLAGS));
//                 if (bwr != sizeof(CRITICAL_FLAGS))
//                 {
//                     syslog(LOG_ERR, "Error writing flag data to MFM\n");
//                     ret = -1;
//                 }
//             }
//             file_close(&fp);
//             syslog(LOG_ERR, "Flassh data written successfully\n", MFM_MAIN_STRPATH, file_name_flag);

//             // printf("")
//         }
//         else
//         {
//             syslog(LOG_ERR, "Unable to open %s%s for writing critical flash data\n", MFM_MAIN_STRPATH, file_name_flag);
//             ret = -1;
//         }
//     }

//     return ret;
// }
#include <time.h>

int compare_flash_data(CRITICAL_FLAGS *flag_data1, CRITICAL_FLAGS *flag_data2)
{
  if (flag_data1->ANT_DEP_STAT == flag_data2->ANT_DEP_STAT &&
      flag_data1->RSV_FLAG == flag_data2->RSV_FLAG &&
      flag_data1->UL_STATE == flag_data2->UL_STATE &&
      flag_data1->OPER_MODE == flag_data2->OPER_MODE &&
      flag_data1->KILL_SWITCH_STAT == flag_data2->KILL_SWITCH_STAT &&
      flag_data1->RST_COUNT == flag_data2->RST_COUNT)
  {
    return 0;
  }
  return -1;
}

// int store_flag_data(CRITICAL_FLAGS *flag_data, uint32_t timer)
// {
//   struct file fp;
//   int bwr;
//   struct timespec start, end;
//   long elapsed_ms;

//   toggle_wdg();
//   int fd = 0;
//   // Check if 2 hours (7200 seconds) have elapsed
//   // if ((timer >= 0 && timer <= 400) || (timer % 7200 < 280 && timer > 280))
//   {
//     // Open internal flash
//     fd = open("/dev/intflash", O_RDWR);
//     if (fd >= 0)
//     {
//       CRITICAL_FLAGS existing_data;

//       // Read existing data from internal flash
//       up_progmem_read(FLAG_DATA_INT_ADDR, &existing_data, sizeof(CRITICAL_FLAGS));

//       // Compare existing data with new flag_data
//       if (compare_flash_data(&existing_data, flag_data) != 0)
//       {
//         // Data is different, proceed with erase and write

//         // Measure time for erase
//         clock_gettime(CLOCK_MONOTONIC, &start);
//         up_progmem_eraseblock(22);
//         clock_gettime(CLOCK_MONOTONIC, &end);

//         elapsed_ms = (end.tv_sec - start.tv_sec) * 1000 + (end.tv_nsec - start.tv_nsec) / 1000000;
//         syslog(LOG_INFO, "Erase time: %ld ms", elapsed_ms);

//         // Measure time for write
//         clock_gettime(CLOCK_MONOTONIC, &start);
//         up_progmem_write(FLAG_DATA_INT_ADDR, flag_data, sizeof(CRITICAL_FLAGS));
//         clock_gettime(CLOCK_MONOTONIC, &end);

//         elapsed_ms = (end.tv_sec - start.tv_sec) * 1000 + (end.tv_nsec - start.tv_nsec) / 1000000;
//         syslog(LOG_INFO, "Write time: %ld ms", elapsed_ms);

//         syslog(LOG_INFO, "Critical flags updated in internal flash.");
//       }
//       else
//       {
//         syslog(LOG_INFO, "No change in critical flags, skipping write to internal flash.");
//       }
//     }
//     else
//     {
//       syslog(LOG_ERR, "Error opening internal flash to store new flag data ... \n");
//     }
//     close(fd);

//     // Open MFM storage
//     int fd1 = open_file_flash(&fp, MFM_MAIN_STRPATH, file_name_flag, O_RDWR);
//     if (fd1 >= 0)
//     {
//       CRITICAL_FLAGS existing_data;
//       if (file_read(&fp, &existing_data, sizeof(CRITICAL_FLAGS)) > 0)
//       {
//         if (compare_flash_data(&existing_data,flag_data) == 0)
//         {
//           printf("Redundant data\n");
//         }
//         else
//         {
//           file_truncate(&fp, sizeof(CRITICAL_FLAGS));

//           // Measure file write time
//           clock_gettime(CLOCK_MONOTONIC, &start);
//           bwr = file_write(&fp, flag_data, sizeof(CRITICAL_FLAGS));
//           clock_gettime(CLOCK_MONOTONIC, &end);

//           elapsed_ms = (end.tv_sec - start.tv_sec) * 1000 + (end.tv_nsec - start.tv_nsec) / 1000000;
//           syslog(LOG_INFO, "MFM Write time: %ld ms", elapsed_ms);

//           if (bwr == 0 || bwr != sizeof(CRITICAL_FLAGS))
//           {
//             syslog(LOG_ERR, "Error in writing flag data to MFM\n Will try once again without verifying...\n");

//             file_truncate(&fp, sizeof(CRITICAL_FLAGS));
//             bwr = file_write(&fp, flag_data, sizeof(CRITICAL_FLAGS));
//             syslog(LOG_INFO, "Size of flag data written to MFM on second attempt: %d", bwr);
//           }
//         }
//       }
//     }
//     else
//     {
//       syslog(LOG_ERR, "Unable to open %s%s for writing critical flash data\n", MFM_MAIN_STRPATH, file_name_flag);
//     }
//     file_close(&fp);
//   }
//   return 0;
// }

int store_flag_data(CRITICAL_FLAGS *flag_data, uint32_t timer)
{
    struct file fp;
    int bwr;
    struct timespec start, end;
    long elapsed_ms;

    toggle_wdg();
    int fd = 0;

    // Check if 2 hours (7200 seconds) have elapsed
    if ((timer >= 0 && timer <= 400) || (timer % 7200 < 280 && timer > 280))
    {
        // Open internal flash
        fd = open("/dev/intflash", O_RDWR);
        if (fd >= 0)
        {
            CRITICAL_FLAGS existing_data;

            // Read existing data from internal flash
            up_progmem_read(FLAG_DATA_INT_ADDR, &existing_data, sizeof(CRITICAL_FLAGS));

            // Compare existing data with new flag_data
            if (memcmp(&existing_data, flag_data, sizeof(CRITICAL_FLAGS)) != 0)
            {
                // Data is different, proceed with erase and write

                // Measure time for erase
                clock_gettime(CLOCK_MONOTONIC, &start);
                up_progmem_eraseblock(22);
                clock_gettime(CLOCK_MONOTONIC, &end);

                elapsed_ms = (end.tv_sec - start.tv_sec) * 1000 + (end.tv_nsec - start.tv_nsec) / 1000000;
                // syslog(LOG_INFO, "Erase time: %ld ms", elapsed_ms);

                // Measure time for write
                clock_gettime(CLOCK_MONOTONIC, &start);
                up_progmem_write(FLAG_DATA_INT_ADDR, flag_data, sizeof(CRITICAL_FLAGS));
                clock_gettime(CLOCK_MONOTONIC, &end);

                elapsed_ms = (end.tv_sec - start.tv_sec) * 1000 + (end.tv_nsec - start.tv_nsec) / 1000000;
                // syslog(LOG_INFO, "Write time: %ld ms", elapsed_ms);

                syslog(LOG_INFO, "Critical flags updated in internal flash.");
            }
            else
            {
                syslog(LOG_INFO, "No change in critical flags, skipping write to internal flash.");
            }
        }
        else
        {
            syslog(LOG_ERR, "Error opening internal flash to store new flag data ... \n");
        }
        close(fd);

        // Open MFM storage
        int fd1 = open_file_flash(&fp, MFM_MAIN_STRPATH, file_name_flag, O_RDWR);
        if (fd1 >= 0)
        {
            CRITICAL_FLAGS existing_data;

            // Read existing data from MFM storage
            file_read(&fp, &existing_data, sizeof(CRITICAL_FLAGS));

            // Compare existing data with new flag_data
            if (memcmp(&existing_data, flag_data, sizeof(CRITICAL_FLAGS)) != 0)
            {
              toggle_wdg();
                // Data is different, proceed with truncate and write

                file_seek(&fp, 0, SEEK_SET);
                bwr = file_write(&fp, flag_data, sizeof(CRITICAL_FLAGS));

                // // Measure file write time
                // clock_gettime(CLOCK_MONOTONIC, &start);
                // clock_gettime(CLOCK_MONOTONIC, &end);

                // elapsed_ms = (end.tv_sec - start.tv_sec) * 1000 + (end.tv_nsec - start.tv_nsec) / 1000000;
                // syslog(LOG_INFO, "MFM Write time: %ld ms", elapsed_ms);

                if (bwr == 0 || bwr != sizeof(CRITICAL_FLAGS))
                {
                    syslog(LOG_ERR, "Error in writing flag data to MFM\n Will try once again without verifying...\n");
                    bwr = file_write(&fp, flag_data, sizeof(CRITICAL_FLAGS));
                    syslog(LOG_INFO, "Size of flag data written to MFM on second attempt: %d", bwr);
                }

                syslog(LOG_INFO, "Critical flags updated in MFM storage.");
            }
            else
            {
                syslog(LOG_INFO, "No change in critical flags, skipping write to MFM storage.");
            }
        }
        else
        {
            syslog(LOG_ERR, "Unable to open %s%s for writing critical flash data\n", MFM_MAIN_STRPATH, file_name_flag);
        }
        file_close(&fp);

        // Open SFM storage
        fd1=-1;
         fd1 = open_file_flash(&fp, SFM_MAIN_STRPATH, file_name_flag, O_CREAT|O_RDWR);
        if (fd1 >= 0)
        {
            CRITICAL_FLAGS existing_data;

            // Read existing data from MFM storage
            file_read(&fp, &existing_data, sizeof(CRITICAL_FLAGS));

            // Compare existing data with new flag_data
            if (memcmp(&existing_data, flag_data, sizeof(CRITICAL_FLAGS)) != 0)
            {
              toggle_wdg();
                // Data is different, proceed with truncate and write

                file_seek(&fp, 0, SEEK_SET);

                // Measure file write time
                // clock_gettime(CLOCK_MONOTONIC, &start);
                bwr = file_write(&fp, flag_data, sizeof(CRITICAL_FLAGS));
                // clock_gettime(CLOCK_MONOTONIC, &end);

                // elapsed_ms = (end.tv_sec - start.tv_sec) * 1000 + (end.tv_nsec - start.tv_nsec) / 1000000;
                // syslog(LOG_INFO, "MFM Write time: %ld ms", elapsed_ms);

                if (bwr == 0 || bwr != sizeof(CRITICAL_FLAGS))
                {
                    syslog(LOG_ERR, "Error in writing flag data to SFM\n Will try once again without verifying...\n");
                    bwr = file_write(&fp, flag_data, sizeof(CRITICAL_FLAGS));
                    syslog(LOG_INFO, "Size of flag data written to SFM on second attempt: %d", bwr);
                }

                syslog(LOG_INFO, "Critical flags updated in SFM storage.");
            }
            else
            {
                syslog(LOG_INFO, "No change in critical flags, skipping write to SFM storage.");
            }
        }
        else
        {
            syslog(LOG_ERR, "Unable to open %s%s for writing critical flash data\n", MFM_MAIN_STRPATH, file_name_flag);
        }
        file_close(&fp);
    }
    return 0;
}

// int check_flag_data(CRITICAL_FLAGS *flags)
// {
//   CRITICAL_FLAGS rd_flags_int = {0};
//   CRITICAL_FLAGS rd_flags_mfm = {0xff};
//   ssize_t read_size_mfm = 0;
//   struct file fp;
//   printf("*********Checking flag data**********\n");
//   toggle_wdg();
//   pthread_mutex_lock(&flash_mutex); // Lock the mutex

//   int fd = open("/dev/intflash", O_RDWR);
//   if (fd >= 0)
//   {
//     syslog(LOG_INFO, "Printing Internal flash flag data.\n");
//     up_progmem_read(FLAG_DATA_INT_ADDR, &rd_flags_int, sizeof(rd_flags_int));
//     print_critical_flag_data(&rd_flags_int);
//     close(fd);
//   }
//   else
//   {
//     syslog(LOG_ERR, "Error opening internal flash\n");
//     return -1;
//   }

//   pthread_mutex_unlock(&flash_mutex); // Lock the mutex

//   int fd1 = file_open(&fp, "/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_RDWR);
//   if (fd1 >= 0)
//   {
//     read_size_mfm = file_read(&fp, &rd_flags_mfm, sizeof(CRITICAL_FLAGS));
//     file_close(&fp);
//   }
//   print_critical_flag_data(&rd_flags_mfm);
//   if (rd_flags_int.ANT_DEP_STAT == 0xff)
//   {
//     if (read_size_mfm != sizeof(CRITICAL_FLAGS))
//     {
//       syslog(LOG_INFO, "Data not available or corrupted in main flash\n");

//       critic_flags.ANT_DEP_STAT = UNDEPLOYED;
//       critic_flags.KILL_SWITCH_STAT = KILL_SW_OFF;
//       critic_flags.OPER_MODE = NRML_MODE;
//       critic_flags.RSV_FLAG = RSV_NOT_RUNNING;
//       critic_flags.UL_STATE = UL_NOT_RX;

//       store_flag_data(&critic_flags,8);
//     }
//     else
//     {
//       syslog(LOG_INFO, "Flag data available in Main Flash Memory Only...Copying data from Main flash memory to internal flash memory\n");

//       critic_flags = rd_flags_mfm;
//       store_flag_data(&critic_flags,0);
//     }
//   }
//   else
//   {
//     if (read_size_mfm != sizeof(CRITICAL_FLAGS))
//     {
//       syslog(LOG_INFO, "Data available in internal flash only\n");

//       critic_flags = rd_flags_int;
//       store_flag_data(&critic_flags,9);
//     }
//     else
//     {
//       syslog(LOG_INFO, "Data available in both main flash and internal flash memories\n");

//       critic_flags = rd_flags_int;
//     }
//   }

//   *flags = critic_flags;
//   printf("*********Checking flag data ended**********\n");

//   return 0;
// }

// int check_flag_data(CRITICAL_FLAGS *flags)
// {
//   CRITICAL_FLAGS rd_flags_int = {0};
//   CRITICAL_FLAGS rd_flags_mfm = {0};
//   ssize_t read_size_mfm = 0;
//   struct file fp;

//   printf("*********Checking flag data**********\n");
//   toggle_wdg();
//   pthread_mutex_lock(&flash_mutex); // Lock the mutex

//   // Read from internal flash
//   int fd = open("/dev/intflash", O_RDWR);
//   if (fd >= 0)
//   {
//     syslog(LOG_INFO, "Reading Internal flash flag data.\n");
//     up_progmem_read(FLAG_DATA_INT_ADDR, &rd_flags_int, sizeof(rd_flags_int));
//     print_critical_flag_data(&rd_flags_int);
//     close(fd);
//   }
//   else
//   {
//     syslog(LOG_ERR, "Error opening internal flash\n");
//     pthread_mutex_unlock(&flash_mutex);
//     return -1;
//   }

//   // Read from MFM
//   int fd1 = file_open(&fp, "/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_RDWR);
//   if (fd1 >= 0)
//   {
//     read_size_mfm = file_read(&fp, &rd_flags_mfm, sizeof(CRITICAL_FLAGS));
//     file_close(&fp);
//   }

//   pthread_mutex_unlock(&flash_mutex); // Unlock the mutex

//   // Print MFM data
//   print_critical_flag_data(&rd_flags_mfm);

//   // Validate and synchronize data
//   if (rd_flags_int.ANT_DEP_STAT == 0xFF)
//   {
//     if (read_size_mfm != sizeof(CRITICAL_FLAGS))
//     {
//       syslog(LOG_INFO, "Data not available or corrupted in main flash\n");
//       memset(&critic_flags, 0, sizeof(CRITICAL_FLAGS)); // Initialize to default values
//       critic_flags.ANT_DEP_STAT = UNDEPLOYED;
//       critic_flags.KILL_SWITCH_STAT = KILL_SW_OFF;
//       critic_flags.OPER_MODE = NRML_MODE;
//       critic_flags.RSV_FLAG = RSV_NOT_RUNNING;
//       critic_flags.UL_STATE = UL_NOT_RX;
//       store_flag_data(&critic_flags, 8);
//     }
//     else
//     {
//       syslog(LOG_INFO, "Copying data from Main flash memory to internal flash memory\n");
//       critic_flags = rd_flags_mfm;
//       store_flag_data(&critic_flags, 9);
//     }
//   }
//   else
//   {
//     if (read_size_mfm != sizeof(CRITICAL_FLAGS))
//     {
//       syslog(LOG_INFO, "Data available in internal flash only\n Copying data from internal flash to MFM");
//       critic_flags = rd_flags_int;
//       store_flag_data(&critic_flags, 9);
//     }
//     else
//     {
//       syslog(LOG_INFO, "Data available in both main flash and internal flash memories\n");
//       if (critic_flags.RST_COUNT == rd_flags_int.RST_COUNT && critic_flags.ANT_DEP_STAT == rd_flags_int.ANT_DEP_STAT && critic_flags.OPER_MODE == rd_flags_int.OPER_MODE && critic_flags.KILL_SWITCH_STAT == rd_flags_int.KILL_SWITCH_STAT)
//       {
//         printf("the data of both memories are same\n");
//       }
//       else
//       {
//         critic_flags = rd_flags_int;
//         store_flag_data(&critic_flags, 9);
//       }
//     }
//   }

//   *flags = critic_flags;
//   printf("*********Checking flag data ended**********\n");
//   return 0;
// }




int check_flag_data(CRITICAL_FLAGS *flags)
{
  CRITICAL_FLAGS rd_flags_int = {0};
  CRITICAL_FLAGS rd_flags_mfm = {0};
  CRITICAL_FLAGS rd_flags_sharedfm = {0};
  ssize_t read_size_mfm = 0;
  ssize_t read_size_sharedfm = 0;
  struct file fp_mfm, fp_sharedfm;

  printf("*********Checking flag data**********\n");
  toggle_wdg();
  pthread_mutex_lock(&flash_mutex); // Lock the mutex

  // Read from internal flash
  int fd = open("/dev/intflash", O_RDWR);
  if (fd >= 0)
  {
    syslog(LOG_INFO, "Reading Internal flash flag data.\n");
    up_progmem_read(FLAG_DATA_INT_ADDR, &rd_flags_int, sizeof(rd_flags_int));
    print_critical_flag_data(&rd_flags_int);
    close(fd);
  }
  else
  {
    syslog(LOG_ERR, "Error opening internal flash\n");
    pthread_mutex_unlock(&flash_mutex);
    return -1;
  }
  pthread_mutex_unlock(&flash_mutex); // Unlock the mutex

  pthread_mutex_lock(&flash_mutex); // Lock the mutex

  // Read from MFM
  int fd1 = file_open(&fp_mfm, "/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_RDWR);
  if (fd1 >= 0)
  {
    read_size_mfm = file_read(&fp_mfm, &rd_flags_mfm, sizeof(CRITICAL_FLAGS));
    file_close(&fp_mfm);
  }

  // Read from SharedFM
  int fd2 = file_open(&fp_sharedfm, "/mnt/fs/sfm/mtd_mainstorage/flags.txt", O_RDWR);
  if (fd2 >= 0)
  {
    read_size_sharedfm = file_read(&fp_sharedfm, &rd_flags_sharedfm, sizeof(CRITICAL_FLAGS));
    file_close(&fp_sharedfm);
  }
  pthread_mutex_unlock(&flash_mutex); // Lock the mutex



  // Print MFM and SharedFM data
  print_critical_flag_data(&rd_flags_mfm);
  print_critical_flag_data(&rd_flags_sharedfm);

  int int_valid = memcmp(&rd_flags_int, &(CRITICAL_FLAGS){0}, sizeof(CRITICAL_FLAGS)) != 0;
  int mfm_valid = memcmp(&rd_flags_mfm, &(CRITICAL_FLAGS){0}, sizeof(CRITICAL_FLAGS)) != 0;
  int sharedfm_valid = memcmp(&rd_flags_sharedfm, &(CRITICAL_FLAGS){0}, sizeof(CRITICAL_FLAGS)) != 0;

  // Determine the majority-consistent data
  if ((compare_flash_data(&rd_flags_int, &rd_flags_mfm) == 0 && int_valid && mfm_valid) ||
      (compare_flash_data(&rd_flags_int, &rd_flags_sharedfm) == 0 && int_valid && sharedfm_valid) && rd_flags_mfm.ANT_DEP_STAT == DEPLOYED)
  {
    syslog(LOG_INFO, "Internal flash data is consistent with at least one source. Using internal flash data.\n");
    critic_flags = rd_flags_int;
  }
  else if (compare_flash_data(&rd_flags_mfm, &rd_flags_sharedfm) == 0 && mfm_valid && sharedfm_valid && rd_flags_mfm.ANT_DEP_STAT == DEPLOYED)
  {
    syslog(LOG_INFO, "MFM and SharedFM are consistent. Using their data.\n");
    critic_flags = rd_flags_mfm;
  }
  else
  {
    syslog(LOG_INFO, "No majority consensus. Preferring internal flash data.\n");
    critic_flags = rd_flags_int;
  }

  // Update all memories with the chosen consistent data
  store_flag_data(&critic_flags, 9);

  *flags = critic_flags;
  printf("*********Checking flag data ended**********\n");
  return 0;
}



void save_critics_flags(const CRITICAL_FLAGS *flags)
{
  printf("*********Saving critical flags data to mfm**********\n");

  int fd = open("/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_WRONLY | O_CREAT, 0666);
  if (fd < 0)
  {
    perror("Failed to open flags.txt for writing");
    return;
  }
  if (write(fd, flags, sizeof(CRITICAL_FLAGS)) != sizeof(CRITICAL_FLAGS))
  {
    perror("Failed to write flags to flags.txt");
  }

  close(fd);
  print_critical_flag_data(flags);
  printf("*********Saving critical flags data to mfm ended**********\n");
}

int load_critics_flags(CRITICAL_FLAGS *flags)
{
  printf("*********load_critics flags Loading critical flags data from int flash started**********\n");

  memset(flags, 0, sizeof(CRITICAL_FLAGS));
  pthread_mutex_lock(&flash_mutex); // Lock the mutex

  int fd;
  ssize_t bytesRead;
  fd = open("/dev/intflash", O_RDONLY);
  if (fd < 0)
  {
    perror("Failed to open /dev/intflash for reading");
    return -1;
  }
  bytesRead = read(fd, flags, sizeof(CRITICAL_FLAGS));
  close(fd);
  pthread_mutex_unlock(&flash_mutex); // Unlock the mutex

  if (bytesRead != sizeof(CRITICAL_FLAGS))
  {
    perror("Failed to read complete flags data");
    return -1;
  }

  printf("*********Loading critical flags data from int flash ended, Started reading mfm**********\n");

  // Process temp if needed
  CRITICAL_FLAGS temp;
  memset(&temp, 0, sizeof(CRITICAL_FLAGS)); // Clear temp structure

  fd = open("/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_RDONLY);
  if (fd < 0)
  {
    perror("Failed to open /mnt/fs/mfm/mtd_mainstorage/flags.txt for reading");
    return -1;
  }
  bytesRead = read(fd, &temp, sizeof(CRITICAL_FLAGS));
  close(fd);

  // Handle data from temp if necessary
  if (bytesRead == sizeof(CRITICAL_FLAGS))
  {
    // Process the read data
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

  printf(" ********************************************\r\n");
  printf(" |   Antenna Deployment Status \t %d \t|\r\n", flags->ANT_DEP_STAT);
  printf(" |   Kill Switch Status        \t %d \t|\r\n", flags->KILL_SWITCH_STAT);
  printf(" |   Operation Mode            \t %d \t|\r\n", flags->OPER_MODE);
  printf(" |   Reservation Table Flag    \t %d \t|\r\n", flags->RSV_FLAG);
  printf(" |   Command uplink status     \t %d \t|\r\n", flags->UL_STATE);
  printf(" |   Reset counter             \t %d \t|\r\n", flags->RST_COUNT);

  printf(" ********************************************\r\n");
}

struct KILL_SW
{
  time_t timestamps[3];
  int count;
};
void add_command_timestamp(struct KILL_SW *kill_sw, time_t timestamp)
{
  if (kill_sw->count < 3)
  {
    kill_sw->timestamps[kill_sw->count++] = timestamp;
  }
  else
  {
    for (int i = 1; i < 3; ++i)
    {
      kill_sw->timestamps[i - 1] = kill_sw->timestamps[i];
    }
    kill_sw->timestamps[3 - 1] = timestamp;
  }
}

int check_command_times(struct KILL_SW *kill_sw)
{
  if (kill_sw->count < 3)
  {
    return 0;
  }

  time_t first_time = kill_sw->timestamps[0];
  time_t last_time = kill_sw->timestamps[3 - 1];

  return (difftime(last_time, first_time) <= 600);
}

int receive_command(struct KILL_SW *kill_sw)
{
  time_t current_time = time(NULL);
  add_command_timestamp(kill_sw, current_time);

  if (check_command_times(kill_sw))
  {
    // save_data();
    // kill_sw.
    // Reset kill_sw after saving data
    kill_sw->count = 0;
    return 0;
  }
  return 1;
}
