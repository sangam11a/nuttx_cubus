
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
 // #include <stdio.h>
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
 #include <sensor/rgb.h>
 #include <sensor/command.h>
 #include <sensor/flash_operation.h>
 
 void Setup();
 #define MFM_MTD_MAINSTORAGE "/mnt/fs/mfm/mtd_mainstorage"
 #define MFM_MTD_MISSION "/mnt/fs/mfm/mtd_mission"
 #define SFM_MTD_MAINSTORAGE "/mnt/fs/sfm/mtd_mainstorage"
 #define SFM_MTD_MISSION "/mnt/fs/sfm/mtd_mission"
 
 #define RESERVATION_CMD "/reservation_command.txt"
 
 #define NB_LOWERHALFS 1
 #define FLASH_DATA_SIZE 80
 int32_t time_counter = 0;
 
 struct SEEK_POINTER
 {
   uint32_t SAT_HEALTH;
   uint32_t SAT_LOG;
   uint32_t CAM_RGB;
   uint32_t CAM_NIR;
   uint32_t EPDM;
   uint32_t ADCS;
 };
 pthread_mutex_t main_flash_mutex =  PTHREAD_MUTEX_INITIALIZER;
 
 // #include <sensor/adc.h>
 static struct work_s work_storage;
 static int8_t count = 0;
 static bool g_storage_manager_started;
 
 void flash_operations();
 void store_sat_health_data(satellite_health_s *sat_health_data, char *pathname);
 void sort_reservation_command(uint16_t file_size, bool reorder);
 int open_file_flash(struct file *file_pointer, char *flash_strpath, char *filename, int open_mode);
 
 
 int store_critical_flag_data(CRITICAL_FLAGS *flag_data)
 {
   struct file fp;
   int bwr;
 
   //printf("\n**************SToring flag data*********\n");
   print_critical_flag_data(flag_data);
   //pthread_mutex_lock(&flash_mutex); // Lock the mutex
   
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
   print_critical_flag_data(flag_data);
   //pthread_mutex_unlock(&flash_mutex); // Lock the mutex
   
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
 
//  void read_and_print_mag_data(void)
//  {
//      int sub_fd;
//      struct reservation_command res, last_res = {0};
//      int fd_reservation;
//      fd_reservation = orb_subscribe(ORB_ID(reservation_command));
//      struct orb_mag_scaled_s mag_data;
//      satellite_health_s satellite_health;
//      bool updated;
 
//      struct pollfd fds2;
//      struct sensor_rgb satHealth;
//      int fd2, ret;
//      fd2 = orb_subscribe_multi(ORB_ID(sensor_rgb), 0);
//      fds2.fd = fd2;
//      fds2.events = POLLIN;
//        sub_fd = orb_subscribe(ORB_ID(orb_mag_scaled));
//          if (sub_fd < 0)
//          {
//              syslog(LOG_ERR, "Failed to subscribe to orb_mag_scaled topic\n");
//              // return;
//          }
    
 
//      while (1)
//      {
//          // if (count % 10 == 0)
//          // {
//          //     sort_reservation_command(1, false);
//          // }
//          count+=10;
       
//          // //printf(
//          //   "read and print amg))))))))))))))))))))))))))))))))))))))\n"
//          // );
 
//          orb_check(fd_reservation, &updated);
//          // //printf("The value of updated is %d\n",updated);
//          if (updated)
//          {
 
//              orb_copy(ORB_ID(reservation_command), fd_reservation, &res);
//              // //printf("the timestamp data is %d %d", res.timestamp,time(NULL));
 
//              // Process only if it's a new command
//              if (memcmp(&res, &last_res, sizeof(res)) != 0 )//res.mcu_id>=3 && res.mcu_id <=5
//              {
//                  //printf("Value of reservation command uORB has been updated\n");
 
//                  // Update the last processed command
//                  // res.latest_time += time_counter;
//                  last_res = res;
//                  printf("The reservation command is %02x %02x %02x %d\n", res.cmd[0], res.cmd[1], res.cmd[2], res.latest_time);
 
                 
         
//                  // if (res.latest_time == 0x00 && res.mcu_id != 0)
//                   uint16_t t = 0x00;
//                   t = (uint16_t)res.cmd[3] << 8 | res.cmd[4];
//                  if (res.mcu_id>=0x01 && res.mcu_id <=0x05 && t <= 35000)                
//                  {
//                      struct file file_ptr;
//                  pthread_mutex_lock(&main_flash_mutex); // Lock the mutex
 
//                      int fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, RESERVATION_CMD, O_CREAT | O_WRONLY | O_APPEND);
//                      if (fd >= 0)
//                      {
//                          ssize_t file_size = file_seek(&file_ptr, 0, SEEK_END);
//                          ssize_t bytes_written = file_write(&file_ptr, &res, sizeof(res));
//                          if (bytes_written > 0)
//                          {
//                              //printf("File size is %zd. Reservation table updated with %zd bytes\n", file_size, bytes_written);
//                              // res.latest_time -= time_counter;
//                          }
//                          // file_close(&file_ptr);
//                      }
//                      file_close(&file_ptr);
//                  pthread_mutex_unlock(&main_flash_mutex); // Lock the mutex
 
//                  }
//              }
//              else
//              {
//                  printf("Duplicate reservation command received. Skipping.\n");
//              }
//          }
 
//          if (count % 90 == 0 | count >= 90)
//          {
//              count = 0;
//              orb_check(sub_fd, &updated);
//              if (updated)
//              {
//                  orb_copy(ORB_ID(orb_mag_scaled), sub_fd, &mag_data);
 
//                  if (poll(&fds2, 1, 3000) > 0)
//                  {
//                      if (fds2.revents & POLLIN)
//                      {
//                          ret = orb_copy_multi(fd2, &satHealth, sizeof(struct sensor_rgb));
//                          if (ret < 0)
//                          {
//                              syslog(LOG_ERR, "ORB copy error, %d \n", ret);
//                              return;
//                          }
//                          else
//                          {
//                              printf("Satellite Health ORB is getting data %d\n", satellite_health.rsv_cmd);
//                          }
//                      }
//                  }
 
//                  satellite_health.accl_x = mag_data.acc_x;
//                  satellite_health.accl_y = mag_data.acc_y;
//                  satellite_health.accl_z = mag_data.acc_z;
 
//                  satellite_health.gyro_x = mag_data.gyro_x;
//                  satellite_health.gyro_y = mag_data.gyro_y;
//                  satellite_health.gyro_z = mag_data.gyro_z;
 
//                  satellite_health.mag_x = mag_data.mag_x;
//                  satellite_health.mag_y = mag_data.mag_y;
//                  satellite_health.mag_z = mag_data.mag_z;
 
//                  store_sat_health_data(&satHealth, MFM_MAIN_STRPATH);
//                  store_sat_health_data(&satHealth, SFM_MAIN_STRPATH);
//                 //  print_satellite_health_data(&satHealth);
//                  // maintain_data_consistency();
//              }
//          }
//          time_counter += 10;
 
//          sleep(10); // Sleep for 1 second
//      }
 
//      orb_unsubscribe(sub_fd);
//      orb_unsubscribe(fd_reservation);
//      orb_unsubscribe(fd2);
//  }
void read_and_print_mag_data(void)
{
    int sub_fd;
    struct reservation_command res, last_res = {0};
    int fd_reservation = orb_subscribe(ORB_ID(reservation_command));
    struct orb_mag_scaled_s mag_data;
    satellite_health_s satellite_health;
    bool updated;
    
    /* Subscribe to sensor_rgb using orb_subscribe (instead of using poll) */
    struct sensor_rgb satHealth;
    int fd2 = orb_subscribe(ORB_ID(sensor_rgb));
    
    sub_fd = orb_subscribe(ORB_ID(orb_mag_scaled));
    if (sub_fd < 0) {
        syslog(LOG_ERR, "Failed to subscribe to orb_mag_scaled topic\n");
        // return; // or handle error as needed
    }
    
    while (1) {
        count += 10;
        
        /* Check for new reservation command data */
        orb_check(fd_reservation, &updated);
        if (updated) {
            orb_copy(ORB_ID(reservation_command), fd_reservation, &res);
            if (memcmp(&res, &last_res, sizeof(res)) != 0) {
                last_res = res;
                printf("The reservation command is %02x %02x %02x %d\n",
                       res.cmd[0], res.cmd[1], res.cmd[2], res.latest_time);
                
                uint16_t t = ((uint16_t)res.cmd[3] << 8) | res.cmd[4];
                if (res.mcu_id >= 0x01 && res.mcu_id <= 0x05 && t <= 35000) {
                    struct file file_ptr;
                    pthread_mutex_lock(&main_flash_mutex); // Lock the mutex
                    int fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, RESERVATION_CMD,
                                               O_CREAT | O_WRONLY | O_APPEND);
                    if (fd >= 0) {
                        ssize_t file_size = file_seek(&file_ptr, 0, SEEK_END);
                        ssize_t bytes_written = file_write(&file_ptr, &res, sizeof(res));
                        // You can add logging here if needed
                    }
                    file_close(&file_ptr);
                    pthread_mutex_unlock(&main_flash_mutex); // Unlock the mutex
                }
            } else {
                printf("Duplicate reservation command received. Skipping.\n");
            }
        }
        
        /* Every 90 count iterations, check for new magnetometer and sensor_rgb data */
        if ((count % 20 == 0) || (count >= 90)) {
            count = 0;
            orb_check(sub_fd, &updated);
            if (updated) {
                orb_copy(ORB_ID(orb_mag_scaled), sub_fd, &mag_data);
                
                /* Instead of polling, check sensor_rgb with orb_check/ orb_copy */
                orb_check(fd2, &updated);
                if (updated) {
                    int ret = orb_copy(ORB_ID(sensor_rgb), fd2, &satHealth);
                    if (ret < 0) {
                        syslog(LOG_ERR, "ORB copy error, %d\n", ret);
                        return;
                    } else {
                        printf("Satellite Health ORB is getting data %d\n", satellite_health.rsv_cmd);
                    }
                }
                
                /* Update satellite health structure with magnetometer data */
                satellite_health.accl_x = mag_data.acc_x;
                satellite_health.accl_y = mag_data.acc_y;
                satellite_health.accl_z = mag_data.acc_z;
                
                satellite_health.gyro_x = mag_data.gyro_x;
                satellite_health.gyro_y = mag_data.gyro_y;
                satellite_health.gyro_z = mag_data.gyro_z;
                
                satellite_health.mag_x = mag_data.mag_x;
                satellite_health.mag_y = mag_data.mag_y;
                satellite_health.mag_z = mag_data.mag_z;
                
                store_sat_health_data(&satHealth, MFM_MAIN_STRPATH);
                store_sat_health_data(&satHealth, SFM_MAIN_STRPATH);
                print_satellite_health_data(&satHealth);
            }
        }
        
        time_counter += 10;
        sleep(10); // Sleep for 10 seconds (adjust as needed)
    }
    
    orb_unsubscribe(sub_fd);
    orb_unsubscribe(fd_reservation);
    orb_unsubscribe(fd2);
}
 
 // void read_and_print_mag_data(void)
 // {
 //   count++;
 //   int sub_fd;
 //   struct reservation_command res;
 //   int fd_reservation;
 //   fd_reservation = orb_subscribe(ORB_ID(reservation_command));
 //   struct orb_mag_scaled_s mag_data;
 //   satellite_health_s satellite_health;
 //   bool updated;
 
 //   struct pollfd fds2;
 //   struct sensor_rgb satHealth;
 //   int fd2, ret;
 //   fd2 = orb_subscribe_multi(ORB_ID(sensor_rgb), 0);
 //   fds2.fd = fd2;
 //   fds2.events = POLLIN;
 
 //   sub_fd = orb_subscribe(ORB_ID(orb_mag_scaled));
 //   // ads = orb_subscibe(ORB_ID(e_ads7953_0));
 //   if (sub_fd < 0)
 //   {
 //     // syslog(LOG_ERR, "Failed to subscribe to orb_mag_scaled topic\n");
 //     return;
 //   }
 //   uint8_t count = 0;
 //   // sleep(30);
 
 //   // sort_reservation_command(1, false);
 
 //   while (1)
 //   {
 //     if (count % 10 == 0)
 //     {
 //       sort_reservation_command(1, false);
 //       // flash_operations();
 //     }
 //     count++;
 //     orb_check(fd_reservation, &updated);
 //     if (updated)
 //     {
 //       // updated = false;
 //       struct file file_ptr;
 //       orb_copy(ORB_ID(reservation_command), fd_reservation, &res);
 //       //printf("Value of reservation command uorb has been updated");
 //       //printf("The reservation command is %02x %02x %02x\n", res.cmd[0], res.cmd[1], res.cmd[2]);
 //       if (res.latest_time == 0x00 && res.mcu_id != 0)
 //       {
 //         int fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, RESERVATION_CMD, O_CREAT | O_WRONLY | O_APPEND);
 //         ssize_t file_size = file_seek(&file_ptr, 0, SEEK_END);
 //         ssize_t bytes_written = file_write(&file_ptr, &res, sizeof(res));
 //         if (bytes_written > 0)
 //         {
 //           //printf("File size is %d.\nReservation table with data size %d has been updated\n", file_size, bytes_written);
 //         }
 //         if (ret = file_syncfs(&file_ptr) < 0)
 //         {
 //           // syslog(LOG_DEBUG, "some issue while synfs closing: %d\n",ret);
 //           file_syncfs(&file_ptr);
 //         }
 //         if (file_close(&file_ptr) < 0)
 //         {
 //           // syslog(LOG_DEBUG, "some issue while synfs closing\n");
 
 //           file_close(&file_ptr);
 //         }
 //         // sort_reservation_command(1);
 //       }
 //       sleep(1);
 //     }
 //     // updated = false;
 //       // //printf("the counter value is %d \n", count);
 
 //     if (count % 90 == 0)
 //     {
 //       count = 1;
 //       orb_check(sub_fd, &updated);
 //       if (updated)
 //       {
 //         orb_copy(ORB_ID(orb_mag_scaled), sub_fd, &mag_data);
 
 //         // edited
 //         if (poll(&fds2, 1, 3000) > 0)
 //         {
 //           if (fds2.revents & POLLIN)
 //           {
 //             ret = orb_copy_multi(fd2, &satHealth, sizeof(struct sensor_rgb));
 //             // print_satellite_health_data(&satHealth);
 //             if (ret < 0)
 //             {
 //               syslog(LOG_ERR, "ORB copy error, %d \n", ret);
 //               return ret;
 //             }
 //             else
 //             {
 //               //printf("Satellite Health ORB is getting data %d\n", satellite_health.rsv_cmd);
 //               // if (count % 12 == 0)
 //               {
 //               }
 //               // //printf("%d %d %d\n",satHealth.accl_x, satHealth.accl_y, satHealth.accl_z);
 //             }
 
 //             // //printf("Temperature: %0.02f \t", mag0.temperature);
 //           }
 //         }
 //         // edited
 
 // #ifdef LOGGING
 //         // syslog(LOG_INFO, "\n\n\r****-------[Storage maanager ]Mag Data:\n");
 
 //         // syslog(LOG_INFO, "  ACCELEROMETER X: %.4f Y: %.4f Z: %.4f\n", mag_data.acc_x, mag_data.acc_y, mag_data.acc_z);
 //         // syslog(LOG_INFO, "  GYROSCOPE X: %.4f Y: %.4f Z: %.4f\n", mag_data.gyro_x, mag_data.gyro_y, mag_data.gyro_z);
 
 //         // syslog(LOG_INFO, "  MAGNETOMETER X: %.4f Y: %.4f Z: %.4f\n", mag_data.mag_x, mag_data.mag_y, mag_data.mag_z);
 //         // syslog(LOG_INFO, "  Temp:  %.4f\n", mag_data.temperature);
 // #endif
 //         satellite_health.accl_x = mag_data.acc_x;
 //         satellite_health.accl_y = mag_data.acc_y;
 //         satellite_health.accl_z = mag_data.acc_z;
 
 //         satellite_health.gyro_x = mag_data.gyro_x;
 //         satellite_health.gyro_y = mag_data.gyro_y;
 //         satellite_health.gyro_z = mag_data.gyro_z;
 
 //         satellite_health.mag_x = mag_data.mag_x;
 //         satellite_health.mag_y = mag_data.mag_y;
 //         satellite_health.mag_z = mag_data.mag_z;
 //         // if (satellite_health.msn_flag != 0x11 && satellite_health.ant_dep_stat == DEPLOYED)
 //         {
 //           store_sat_health_data(&satHealth, MFM_MAIN_STRPATH);
 //           print_satellite_health_data(&satHealth);
 //         }
 //         // store_sat_health_data(&satellite_health, SFM_MAIN_STRPATH);
 //       }
 //       maintain_data_consistency();
 //     }
 
 //     sleep(1); // Sleep for 500 ms
 //   }
 
 //   orb_unsubscribe(sub_fd);
 // }
 
 int storage_manager_daemon(int argc, FAR char *argv[])
 {
   g_storage_manager_started = true;
   read_and_print_mag_data();
   return 0;
 }
 
 int main(int argc, FAR char *argv[])
 {
   int ret;
 
   //printf("[storage_manager] Starting task.\n");
   if (g_storage_manager_started)
   {
     //printf("[storage_manager] Task already started.\n");
     return EXIT_SUCCESS;
   }
 
   // ret = task_create("storage_manager_daemon", SCHED_PRIORITY_DEFAULT, CONFIG_CUSTOM_APPS_STORAGE_MANAGER_STACKSIZE, storage_manager_daemon, NULL);
   ret = task_create("storage_manager_daemon", 90, 8078, storage_manager_daemon, NULL);
 
   if (ret < 0)
   {
     int errcode = errno;
     //printf("[storage_manager] ERROR: Failed to start storage_manager_daemon: %d\n", errcode);
     return EXIT_FAILURE;
   }
   ret = task_create("flash_operations", SCHED_PRIORITY_DEFAULT, 8078, flash_operations, NULL);
 
   if (ret < 0)
   {
     int errcode = errno;
     //printf("[storage_manager] ERROR: Failed to start flash_operations: %d\n", errcode);
     return EXIT_FAILURE;
   }
 
   //printf("[storage_manager] storage_manager_daemon started\n");
   return EXIT_SUCCESS;
 }
 
 void store_sat_health_data(satellite_health_s *sat_health_data, char *pathname)
 {
   struct file file_p;
   int ret;
   // if(strcmp(pathname, SFM_MAIN_STRPATH) == 1){
   //   gpio_write(GPIO_MUX_EN, false);
 
   //   gpio_write(GPIO_SFM_MODE, false);
   // }
   pthread_mutex_lock(&main_flash_mutex);
   // TODO: discuss and figure out if we need to set limit to size of file and truncate contents once the file size limit is reached ...
   if (open_file_flash(&file_p, pathname, file_name_sat_health, O_CREAT | O_WRONLY | O_APPEND) >= 0)
   {
     // uint8_t data_temp[];
     ssize_t bytes_written = file_write(&file_p, sat_health_data, sizeof(satellite_health_s));
     if (bytes_written > 0)
     {
       syslog(LOG_INFO, "Satellite Health data write Successful.\nData Len: %d.\n", bytes_written);
       file_close(&file_p);
     }
     else
     {
       // syslog(LOG_INFO, "Write Failure.\n");
     }
     // file_close(&file_p);
 
     if (ret = file_syncfs(&file_p) < 0)
     {
       // syslog(LOG_DEBUG, "some issue while synfs closing: %d\n",ret);
       file_syncfs(&file_p);
     }
     if (file_close(&file_p) < 0)
     {
       // syslog(LOG_DEBUG, "some issue while synfs closing\n");
 
       file_close(&file_p);
     }
   }
   else
   {
     file_close(&file_p);
     syslog(LOG_ERR, "Error opening file path %s to write satellite health data..\n", pathname);
   }
   pthread_mutex_unlock(&main_flash_mutex);
   CRITICAL_FLAGS critic_flags;
   critic_flags.ANT_DEP_STAT = sat_health_data->ant_dep_stat;
   critic_flags.KILL_SWITCH_STAT= sat_health_data->kill_switch;
   critic_flags.OPER_MODE= sat_health_data->oper_mode;
   critic_flags.RSV_FLAG= sat_health_data->rsv_flag;
   critic_flags.UL_STATE= sat_health_data->ul_state;
   critic_flags.RST_COUNT = sat_health_data->rst_counter;
 
   pthread_mutex_lock(&main_flash_mutex);
     // if(strcpy(pathname, MFM_MAIN_STRPATH) == 0)
     { 
       //printf("the full path is :%s\n",pathname);
       int fd1 = open_file_flash(&file_p, pathname, file_name_flag, O_RDWR);
       if (fd1 >= 0)
       {
         // file_truncate(&file_p, sizeof(CRITICAL_FLAGS));
         ssize_t bwr = file_write(&file_p, &critic_flags, sizeof(CRITICAL_FLAGS));
         if (bwr != sizeof(CRITICAL_FLAGS))
         {
           syslog(LOG_ERR, "Error in writing flag data to MFM\n");
         }
         // file_sync(&file_p);
         file_close(&file_p);
         pthread_mutex_unlock(&main_flash_mutex);
 
       }
       else
       {
         syslog(LOG_ERR, "Unable to open %s%s for writing critical flash data\n", pathname, file_name_flag);
         pthread_mutex_unlock(&main_flash_mutex);
       }
     }
   syslog(LOG_DEBUG, "\n-----Storing data to flash-----\n");  
 
 
   // if(strcmp(pathname, SFM_MAIN_STRPATH) == 1){
 
   //   gpio_write(GPIO_SFM_MODE, true);
   // }
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
    //  syslog(LOG_INFO, "Opened file: %s ...\n", path);
   }
   return fd;
 }
 
 void flash_operations()
 {
   struct command command_ops, temp_command_ops;
   int updated = 0;
   int flash_fd = orb_subscribe_multi(ORB_ID(command), 10);
   temp_command_ops.address = 1234567890;
   temp_command_ops.num_of_packets = 65533;
   // orb_check(flash_fd, &updated);
   // if (updated)
   while (1)
   {
     orb_check(flash_fd, &updated);
     if (updated)
     {
       updated = false;
       orb_copy_multi(flash_fd, &command_ops, sizeof(struct command));
    
       if (temp_command_ops.address != command_ops.address && temp_command_ops.num_of_packets != command_ops.num_of_packets)
       {
         switch (command_ops.command)
         {
         case 0xca:
           truncate_text_file(command_ops.path); //TODO need to reform this
           //printf("-----Trucate text file called \n");
           break;
         case 0x1d:
           // FLASH_OPERATION = true;
           // MISSION_STATUS.FLASH_OPERATION = true;
           temp_command_ops.address = command_ops.address;
           temp_command_ops.num_of_packets = command_ops.num_of_packets;
           //printf("***************************************************************\n");
           //printf("command_ops.address %d\n command_ops.num_of_packets %d\n", command_ops.address, command_ops.num_of_packets);
           //printf("***************************************************************\n");
           send_data_uorb(command_ops.path, command_ops.address, command_ops.num_of_packets, command_ops.pkt_type);
         }
       }
       else
       {
         temp_command_ops.address = 1234567890;
         temp_command_ops.num_of_packets = 65533;
       }
     }
     sleep(1);
     orb_check(flash_fd, &updated);
   }
   orb_unsubscribe(ORB_ID(command));
 }
 
 void Setup()
 {
   // int fd = 0;
   struct file flp1, flp2, flp3, flp4;
 
 //pthread_mutex_lock(&main_flash_mutex); // Lock the mutex
   char filename[][30] = {"/flags.txt", "/satHealth.txt", "/reservation_command.txt","/time.txt","/epdm.txt", "/cam_rgb.txt", "/adcs.txt",  "/cam_nir.txt", "/digipeater.txt", "/adcs_logs.txt", "/epdm_logs.txt", "/cam_rgb_logs.txt", "/cam_nir_logs.txt"};
   int fd ;
   
   for (int i = 0; i < sizeof(filename) / sizeof(filename[0]); i++)
   {
     if(i <= 3){
       fd = open_file_flash(&flp1, MFM_MAIN_STRPATH, filename[i], O_CREAT); 
       if (fd < 0)
       {
        //  syslog(LOG_ERR, "Could not create file named sat_health... \n");
       }
       file_close(&flp1);
       fd = open_file_flash(&flp1, SFM_MAIN_STRPATH, filename[i], O_CREAT); 
       if (fd < 0)
       {
        //  syslog(LOG_ERR, "Could not create file named sat_health... \n");
       }
       file_close(&flp1);
     }
     else if(i==4){
       fd = open_file_flash(&flp1, MFM_MSN_STRPATH, filename[i], O_CREAT); 
       if (fd < 0)
       {
        //  syslog(LOG_ERR, "Could not create file named sat_health... \n");
       }
       file_close(&flp1);
       fd = open_file_flash(&flp1, SFM_MAIN_STRPATH, filename[i], O_CREAT); 
       if (fd < 0)
       {
        //  syslog(LOG_ERR, "Could not create file named sat_health... \n");
       }
       file_close(&flp1);
     }
     else{
       fd = open_file_flash(&flp1, MFM_MSN_STRPATH, filename[i], O_CREAT); 
       if (fd < 0)
       {
        //  syslog(LOG_ERR, "Could not create file named sat_health... \n");
       }
       file_close(&flp1);
       fd = open_file_flash(&flp1, SFM_MSN_STRPATH, filename[i], O_CREAT); 
       if (fd < 0)
       {
        //  syslog(LOG_ERR, "Could not create file named sat_health... \n");
       }
       file_close(&flp1);
     }
     
   }
   // fd = open_file_flash(&flp1, MFM_MAIN_STRPATH, file_name_sat_health, O_CREAT);
   // if (fd < 0)
   // {
   //   syslog(LOG_ERR, "Could not create file named sat_health... \n");
   // }
   // file_close(&flp1);
 
   // fd = open_file_flash(&flp2, MFM_MAIN_STRPATH, file_name_flag, O_CREAT);
   // if (fd < 0)
   // {
   //   syslog(LOG_ERR, "Could not create flags.txt data file ... \n");
   // }
   // file_close(&flp2);
 
   //   fd = open_file_flash(&flp2, MFM_MAIN_STRPATH, "reservation_command", O_CREAT);
   // if (fd < 0)
   // {
   //   syslog(LOG_ERR, "Could not create flags.txt data file ... \n");
   // }
   // file_close(&flp2);
 
   //   fd = open_file_flash(&flp2, MFM_MAIN_STRPATH, "/seek_pointer.txt", O_CREAT);
   // if (fd < 0)
   // {
   //   syslog(LOG_ERR, "Could not create flags.txt data file ... \n");
   // }
   // file_close(&flp2);
 
   // fd = open_file_flash(&flp3, MFM_MSN_STRPATH, file_name_cam_msn, O_CREAT);
   // if (fd < 0)
   // {
   //   syslog(LOG_ERR, "could not create cam.txt msn file ...\n");
   // }
   // file_close(&flp3);
 
   // fd = open_file_flash(&flp4, MFM_MSN_STRPATH, file_name_epdm_msn, O_CREAT);
   // if (fd < 0)
   // {
   //   syslog(LOG_ERR, "Could not create epdm.txt msn file\n");
   // }
   // file_close(&flp4);
 //pthread_mutex_unlock(&main_flash_mutex); // Lock the mutex
 
 
   syslog(LOG_INFO, "Checking initial flag data...\n");
   check_flag_data();
   print_critical_flag_data(&critic_flags);
 }
 
 void sort_reservation_command(uint16_t file_size, bool reorder)
 {
  struct file file_ptr;
         uint16_t temp = 0;
 
         int fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, "/reservation_command.txt", O_RDONLY);
         struct reservation_command res_temp[16];
          file_size = file_seek(&file_ptr, 0, SEEK_END);
 
         for (int i = 0; i < (file_size / sizeof(struct reservation_command)); i++)
         {
             file_seek(&file_ptr, temp, SEEK_SET);
             temp += sizeof(struct reservation_command);
             file_read(&file_ptr, &res_temp[i], sizeof(struct reservation_command));
         }
         file_close(&file_ptr);
 
         int n = file_size / sizeof(struct reservation_command);
 
         for (int i = 1; i < n - 1; i++)
         {
             for (int j = 1; j < n - i; j++)
             {
                 uint16_t t1 = (uint16_t)res_temp[j].cmd[3] << 8 | res_temp[j].cmd[4];
                 uint16_t t2 = (uint16_t)res_temp[j + 1].cmd[3] << 8 | res_temp[j + 1].cmd[4];
                 if (t1 > t2)
                 {
                     struct reservation_command temp = res_temp[j];
                     res_temp[j] = res_temp[j + 1];
                     res_temp[j + 1] = temp;
                 }
             }
         }
 
         for (int i = 1; i < n; i++)
         {
             //printf("Number of res command read is %d and temp is %d\n", i + 1, temp);
             //printf("_________________Sorted________________________\n");
             printf("\nMCU id : %02x Cmd[0] :%02x , Cmd[1]:%02x ,Cmd[2]:%02x, Cmd[3]:%02x Time[0]:%02x Time[1]:%d executed:%d\n",
                    res_temp[i].mcu_id, res_temp[i].cmd[0], res_temp[i].cmd[1], res_temp[i].cmd[2], res_temp[i].cmd[3], res_temp[i].cmd[4], res_temp[i].cmd[5], res_temp[i].executed);
             //printf("__________________Sorted_______________________\n");
         }
 
         if (n == 1)
         {
             fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, "/reservation_command.txt", O_TRUNC);
         }
         else
         {
             fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, "/reservation_command.txt", O_TRUNC | O_WRONLY | O_APPEND);
             for (int i = 1; i < n; i++)
             {
                 file_write(&file_ptr, &res_temp[i], sizeof(struct reservation_command));
             }
         }
         file_close(&file_ptr);
 }
 
 void maintain_data_consistency()
 {
     struct file mfm_file_pointer, sfm_file_pointer;
     int mfm_fd, sfm_fd;
     uint32_t mfm_read = 0, sfm_read = 0, seek_ptr = 0;
     uint8_t temp[2000];
     char filename[][30] = {"/flags.txt", "/satHealth.txt", "/reservation_command.txt","/time.txt","/epdm.txt", "/cam_rgb.txt", "/adcs.txt",  "/cam_nir.txt", "/digipeater.txt", "/adcs_logs.txt", "/epdm_logs.txt", "/cam_rgb_logs.txt", "/cam_nir_logs.txt"};
   
     pthread_mutex_lock(&main_flash_mutex);
     
     for (int i = 0; i < sizeof(filename) / sizeof(filename[0]); i++)
     {
       if(i<=3){
         mfm_fd = open_file_flash(&mfm_file_pointer, MFM_MAIN_STRPATH, filename[i], O_CREAT |O_RDWR | O_APPEND);
         sfm_fd = open_file_flash(&sfm_file_pointer, SFM_MAIN_STRPATH, filename[i], O_CREAT| O_RDWR | O_APPEND);
         
       }
       else if(i==4){
         mfm_fd = open_file_flash(&mfm_file_pointer, MFM_MSN_STRPATH, filename[i], O_CREAT |O_RDWR | O_APPEND);
         sfm_fd = open_file_flash(&sfm_file_pointer, SFM_MAIN_STRPATH, filename[i], O_CREAT| O_RDWR | O_APPEND);
         
       }
       else{
          mfm_fd = open_file_flash(&mfm_file_pointer, MFM_MSN_STRPATH, filename[i], O_CREAT| O_RDWR | O_APPEND);
         sfm_fd = open_file_flash(&sfm_file_pointer, SFM_MSN_STRPATH, filename[i], O_CREAT| O_RDWR | O_APPEND);
         
       }
        
         if (mfm_fd < 0 || sfm_fd < 0) {
             if (mfm_fd >= 0) file_close(&mfm_file_pointer);
             if (sfm_fd >= 0) file_close(&sfm_file_pointer);
             continue;
         }
 
         mfm_read = file_seek(&mfm_file_pointer, 0, SEEK_END);
         sfm_read = file_seek(&sfm_file_pointer, 0, SEEK_END);
 
         if (sfm_read > mfm_read) {
             seek_ptr = mfm_read;
             file_seek(&sfm_file_pointer, seek_ptr, SEEK_SET);
             
             while (seek_ptr < sfm_read) {
                 uint32_t chunk_size = (sfm_read - seek_ptr > sizeof(temp)) ? sizeof(temp) : sfm_read - seek_ptr;
                 
                 ssize_t read_bytes = file_read(&sfm_file_pointer, temp, chunk_size);
                 if (read_bytes > 0) {
                     ssize_t written = file_write(&mfm_file_pointer, temp, read_bytes);
                     if (written != read_bytes) {
                         //printf("Error writing to MFM for %s\n", filename[i]);
                     } else {
                         //printf("Copied %d bytes from SFM to MFM for %s\n", written, filename[i]);
                     }
                 } else {
                     //printf("Error reading from SFM for %s\n", filename[i]);
                     break;
                 }
                 
                 seek_ptr += chunk_size;
             }
         }
         else if (mfm_read > sfm_read) {
             seek_ptr = sfm_read;
             file_seek(&mfm_file_pointer, seek_ptr, SEEK_SET);
             
             while (seek_ptr < mfm_read) {
                 uint32_t chunk_size = (mfm_read - seek_ptr > sizeof(temp)) ? sizeof(temp) : mfm_read - seek_ptr;
                 
                 ssize_t read_bytes = file_read(&mfm_file_pointer, temp, chunk_size);
                 if (read_bytes > 0) {
                     ssize_t written = file_write(&sfm_file_pointer, temp, read_bytes);
                     if (written != read_bytes) {
                         //printf("Error writing to SFM for %s\n", filename[i]);
                     } else {
                         //printf("Copied %d bytes from MFM to SFM for %s\n", written, filename[i]);
                     }
                 } else {
                     //printf("Error reading from MFM for %s\n", filename[i]);
                     break;
                 }
                 
                 seek_ptr += chunk_size;
             }
         }
 
         file_close(&mfm_file_pointer);
         file_close(&sfm_file_pointer);
     }
     
     pthread_mutex_unlock(&main_flash_mutex);
 }
 
 
 // void print_seek_pointer()
 // {
 //   struct file fp_seek;
 //   struct SEEK_POINTER seek_pointer;
 //   if (open_file_flash(&fp_seek, MFM_MAIN_STRPATH, "/seek_pointer.txt", O_RDONLY) >= 0)
 //   {
 //     int fd_seek = file_read(&fp_seek, &seek_pointer, sizeof(struct SEEK_POINTER));
 //     if (fd_seek >= 0)
 //     {
 //       //printf("\n----------------------------------------------------------\nSAT_HEALTH:%d\n SAT_LOG:%d\n CAM_RGB:%d\n CAM_NIR:%d\n EPDM:%d\n ADCS:%d\n----------------------------------------------------------\n",
 //              seek_pointer.SAT_HEALTH, seek_pointer.SAT_LOG, seek_pointer.CAM_RGB,
 //              seek_pointer.CAM_NIR, seek_pointer.EPDM, seek_pointer.ADCS);
 //     }
 //   }
 //   file_close(&fp_seek);
 // }
 
 void print_seek_pointer() {
     struct SEEK_POINTER seek_pointer = {0};
     struct file fp;
 
     if (open_file_flash(&fp, MFM_MAIN_STRPATH, "/seek_pointer.txt", O_RDONLY) >= 0) {
         if (file_read(&fp, &seek_pointer, sizeof(seek_pointer)) >= sizeof(seek_pointer)) {
             //printf("\n------------------- Seek Pointer -------------------\n");
             //printf("SAT_HEALTH: %d, SAT_LOG: %d, CAM_RGB: %d, CAM_NIR: %d, EPDM: %d, ADCS: %d\n", seek_pointer.SAT_HEALTH, seek_pointer.SAT_LOG, seek_pointer.CAM_RGB,  seek_pointer.CAM_NIR, seek_pointer.EPDM, seek_pointer.ADCS);
             //printf("---------------------------------------------------\n");
         } else {
             //printf("Error: Failed to read seek pointer file\n");
         }
         file_close(&fp);
     } else {
         //printf("Error: Failed to open seek pointer file\n");
     }
 }
 
 void send_data_uorb(char path[200], uint32_t address, int16_t num_of_packets, uint8_t packet_type)
 {
   int adc_instance  = 10;
   int16_t temp_packets = num_of_packets;
   struct flash_operation flash = {'\0'};
   struct file fp_seek;
   // uint16_t test[8];
   struct SEEK_POINTER seek_pointer;
   struct file fp;
 pthread_mutex_lock(&main_flash_mutex); // Lock the mutex
 
   int fd = file_open(&fp, path, O_RDONLY);
   // int fd_seek = file_open(&fp_seek, "/mnt/fs/mfm/mtd_mainstorage/seek_pointer.txt", O_RDWR | O_TRUNC);
   // if (file_seek(&fp_seek, 0, SEEK_END) == 0)
   // {
   //   file_write(&fp_seek, &seek_pointer, sizeof(struct SEEK_POINTER));
   // }
   // if (fd_seek >= 0)
   // {
   //   //printf("The seek pointer file has opened\n");
   // }
   // // fd_seek = file_read(&fp_seek, &seek_pointer, sizeof(seek_pointer));
   // // if (fd_seek >= 0)
   // // {
   // //   //printf("SAT_HEALTH:%d\n SAT_LOG:%d\n CAM_RGB:%d\n CAM_NIR:%d\n EPDM:%d\n ADCS:%d\n",
   // //          seek_pointer.SAT_HEALTH, seek_pointer.SAT_LOG, seek_pointer.CAM_RGB,
   // //          seek_pointer.CAM_NIR, seek_pointer.EPDM, seek_pointer.ADCS);
   // // }
   // file_close(&fp_seek);
   // print_seek_pointer();
   int raw_afd = orb_advertise_multi_queue_persist(ORB_ID(flash_operation), &flash,
                                                   &adc_instance, sizeof(struct flash_operation));
   uint32_t count = 0, pkt = 0;
   flash.packet_type = packet_type;
   // while (1)
   {
     if (raw_afd < 0)
     {
       syslog(LOG_ERR, "Raw ADC advertise failed. %i\n", raw_afd);
     }
     else
     {
 
       {
         printf("\nData sent :\nTimestamp : %d \nPacket Type:%d \nPacket Number:%d \n",
                flash.timestamp,
                flash.packet_type,
                flash.packet_number);
 
         int64_t size_of_file = file_seek(&fp, 0, SEEK_END);
         //printf("The size of file named %s is : %d\n", path, size_of_file);
         uint8_t data[80] = {'\0'};
         if (fd >= 0)
         {
           count = address;
           if (num_of_packets == 0)
           {
             num_of_packets = 1;
           }
           //printf("the value of count seek pointer is %d %d\n", address);
           do
           {
             if (size_of_file > 0 && count <= size_of_file)
             {
               flash.timestamp = (uint64_t)time(NULL);
               // //printf("file has been opened with name %s\n", path);
               // //printf("\n----------------------------\n");
               pkt++;
               // TODO:read data at once and send it accordingly instead
 
               file_seek(&fp, count, SEEK_SET);
               // file_read(&fp, &flash.data, sizeof(data));
               file_read(&fp, data, sizeof(data));
               // flash.data = data;
               for (int i = 0; i < 80; i++)
               {
                 // //printf("%02x ", data[i]);
                 flash.data[i] = data[i];
               }
               flash.packet_type = packet_type;
               flash.packet_number = pkt;
              //  printf("cc: %d num_of_pkt:%d   Packet number:%d\n",count,num_of_packets, flash.packet_number);
               // //printf("\n----------------------------\n");
               count += 80;
 
               if (OK != orb_publish(ORB_ID(flash_operation), raw_afd, &flash))
               {
                 syslog(LOG_ERR, "Orb Publish failed\n");
               }
               else
               {
                 syslog(LOG_ERR, "Orb data published\n");
               }
               // if(num_of_packets == pkt) break;
               num_of_packets -= 1;
             }
             else
             {
               break;
             }
            // sleep(1);
            // sleep(1);
            usleep(1700000);
            //  sleep(2.5);//TODO Need to look on the packets so that it is not missed
           } while (num_of_packets > 0);
         }
         else
         {
           // flash.data[0] = 0x53;
           flash.packet_number = 0x01;
           flash.packet_type = packet_type;
           flash.data[0] = 0x04;
           flash.data[1] = 0xac;
           flash.data[2] = 0x04;
           flash.data[3] = 0x63;
           flash.data[4] = 0x62;
           flash.data[84] = 0x7e;
           flash.data[85] = 0x7e;
 
           if (OK != orb_publish(ORB_ID(flash_operation), raw_afd, &flash))
           {
             syslog(LOG_ERR, "Orb Publish failed\n");
           }
         }
         file_close(&fp);
     pthread_mutex_unlock(&main_flash_mutex); // Lock the mutex
 
         struct SEEK_POINTER temp;
     pthread_mutex_lock(&main_flash_mutex); // Lock the mutex
         if(temp_packets !=0)
        { fd = file_open(&fp, "/mnt/fs/mfm/mtd_mainstorage/seek_pointer.txt", O_CREAT | O_RDWR);
         if (fd >= 0)
         {
           if (file_read(&fp, &temp, sizeof(struct SEEK_POINTER)) >= 0)
           {
             file_close(&fp);
             // count = file_seek(&fp, 0, SEEK_CUR);
             count = count + address;
             if (strcmp(path, "/mnt/fs/mfm/mtd_mainstorage/satHealth.txt") == 0)
             {
               temp.SAT_HEALTH = (uint32_t) count;
             }
             else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/cam_rgb.txt") == 0)
             {
               temp.CAM_RGB = (uint32_t) count;
             }
             else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/epdm.txt") == 0)
             {
               temp.EPDM = (uint32_t) count;
             }
             else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/cam_nir.txt") == 0)
             {
               temp.CAM_NIR = (uint32_t) count;
             }
             else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/adcs.txt") == 0)
             {
               temp.ADCS = (uint32_t) count;
             }
             // ssize_t writeBytes = file_write(&fp, &temp, sizeof(struct SEEK_POINTER));
             // //printf()
             // if (writeBytes >= 0)
             {
               //printf("seek pointer updated\n");
                //printf("\n----------------------------------------------------------\nSAT_HEALTH:%d\n SAT_LOG:%d\n CAM_RGB:%d\n CAM_NIR:%d\n EPDM:%d\n ADCS:%d\n----------------------------------------------------------\n",temp.SAT_HEALTH, temp.SAT_LOG, temp.CAM_RGB, temp.CAM_NIR, temp.EPDM, temp.ADCS);
             }
             // else{
             //   //printf("Seek operation updation failed\n");
             // }
           }
         }
         close(fd);}
     pthread_mutex_unlock(&main_flash_mutex); // Lock the mutex
 
         // file_close(&fp);
         update_seek_pointer(path, address, count, temp);
 
         print_seek_pointer();
 
       }
     }
   }
   // orb_unadvertise(raw_afd);
 }
 
 
 // // Function to update the seek pointer
 // void update_seek_pointer(const char *path, uint32_t address, uint32_t count) {
 //     struct SEEK_POINTER temp;
 //     struct file fp; // Assuming `struct file` is defined for file operations
 //     int fd = file_open(&fp, "/mnt/fs/mfm/mtd_mainstorage/seek_pointer.txt", O_CREAT | O_RDWR);
 
 //     if (fd >= 0) {
 //         // Read the current seek pointer values
 //         if (file_read(&fp, &temp, sizeof(struct SEEK_POINTER)) >= sizeof(struct SEEK_POINTER)) {
 //             // Update `count` based on address
 //             count += address;
 
 //             // Update the appropriate field in the SEEK_POINTER structure
 //             if (strcmp(path, "/mnt/fs/mfm/mtd_mainstorage/satHealth.txt") == 0) {
 //                 temp.SAT_HEALTH = count;
 //             } else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/cam_rgb.txt") == 0) {
 //                 temp.CAM_RGB = count;
 //             } else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/epdm.txt") == 0) {
 //                 temp.EPDM = count;
 //             } else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/cam_nir.txt") == 0) {
 //                 temp.CAM_NIR = count;
 //             } else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/adcs.txt") == 0) {
 //                 temp.ADCS = count;
 //             } else {
 //                 //printf("Error: Unknown path\n");
 //                 file_close(&fp);
 //                 return; // Exit on unknown path
 //             }
 
 //             // Write the updated structure back to the file
 //             ssize_t writeBytes = file_write(&fp, &temp, sizeof(struct SEEK_POINTER));
 //             if (writeBytes >= sizeof(struct SEEK_POINTER)) {
 //                 //printf("Seek pointer updated successfully\n");
 //                 //printf("Updated values:\n");
 //                 //printf("SAT_HEALTH=%d, SAT_LOG=%d, CAM_RGB=%d, CAM_NIR=%d, EPDM=%d, ADCS=%d\n",
 //                        temp.SAT_HEALTH, temp.SAT_LOG, temp.CAM_RGB, temp.CAM_NIR, temp.EPDM, temp.ADCS);
 //             } else {
 //                 //printf("Error: Failed to write updated seek pointer\n");
 //             }
 //         } else {
 //             //printf("Error: Failed to read seek pointer\n");
 //         }
 //         file_close(&fp);
 //     } else {
 //         //printf("Error: Failed to open seek_pointer file\n");
 //     }
 // }
 void update_seek_pointer(const char *path, uint32_t address, uint32_t count, struct SEEK_POINTER seek_pointer) {
     // struct SEEK_POINTER seek_pointer = {0};
     struct file fp;
 
     int fd = file_open(&fp, "/mnt/fs/mfm/mtd_mainstorage/seek_pointer.txt", O_RDWR);
     if (fd < 0) {
         //printf("Error: Failed to open seek pointer file\n");
         return;
     }
 
     file_read(&fp, &seek_pointer, sizeof(seek_pointer));
 
     count += address;
     if (strcmp(path, "/mnt/fs/mfm/mtd_mainstorage/satHealth.txt") == 0) {
         seek_pointer.SAT_HEALTH = count;
     } else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/cam_rgb.txt") == 0) {
         seek_pointer.CAM_RGB = count;
     } else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/epdm.txt") == 0) {
         seek_pointer.EPDM = count;
     } else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/cam_nir.txt") == 0) {
         seek_pointer.CAM_NIR = count;
     } else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/adcs.txt") == 0) {
         seek_pointer.ADCS = count;
     } else {
         printf("Error: Unknown path: %s\n", path);
         file_close(&fp);
        //  return;
     }
 
     file_seek(&fp, 0, SEEK_SET);
     if (file_write(&fp, &seek_pointer, sizeof(seek_pointer)) < 0) {
         printf("Error: Failed to update seek pointer\n");
     } else {
         //printf("Seek pointer updated successfully\n");
         print_seek_pointer();
     }
 
     file_close(&fp);
 }
 
 // void send_data_uorb(char path[200], uint32_t address, uint16_t num_of_packets, uint8_t packet_type)
 // {
 //   int adc_instance = 10;
 //   struct SEEK_POINTER
 //   {
 //     uint16_t SAT_HEALTH;
 //     uint16_t SAT_LOG;
 //     uint16_t CAM_RGB;
 //     uint16_t CAM_NIR;
 //     uint16_t EPDM;
 //     uint16_t ADCS;
 //   };
 //   struct flash_operation flash = {'\0'};
 //   struct file fp_seek;
 //   struct SEEK_POINTER seek_pointer;
 //   int fd_seek = file_open(&fp_seek, "/mnt/fs/mfm/mtd_mainstorage/seek_pointer.txt", O_RDWR | O_TRUNC);
 //   if (file_seek(&fp_seek, 0, SEEK_END) == 0)
 //   {
 //     file_write(&fp_seek, &seek_pointer, sizeof(struct SEEK_POINTER));
 //   }
 //   if (fd_seek >= 0)
 //   {
 //     //printf("The seek pointer file has opened\n");
 //   }
 //   fd_seek = file_read(&fp_seek, &seek_pointer, sizeof(seek_pointer));
 //   if (fd_seek >= 0)
 //   {
 //     //printf("SAT_HEALTH:%d\n SAT_LOG:%d\n CAM_RGB:%d\n CAM_NIR:%d\n EPDM:%d\n ADCS:%d\n",
 //            seek_pointer.SAT_HEALTH, seek_pointer.SAT_LOG, seek_pointer.CAM_RGB,
 //            seek_pointer.CAM_NIR, seek_pointer.EPDM, seek_pointer.ADCS);
 //   }
 //   int raw_afd = orb_advertise_multi_queue_persist(ORB_ID(flash_operation), &flash,
 //                                                   &adc_instance, sizeof(struct flash_operation));
 //   int count = 0, pkt = 0;
 //   flash.packet_type = packet_type;
 //   {
 //     if (raw_afd < 0)
 //     {
 //       syslog(LOG_ERR, "Raw ADC advertise failed. %i\n", raw_afd);
 //     }
 //     else
 //     {
 //       //printf("\nData sent :\nTimestamp : %d \nPacket Type:%d \nPacket Number:%d \n",
 //              flash.timestamp,
 //              flash.packet_type,
 //              flash.packet_number);
 //       struct file fp;
 //       int fd = file_open(&fp, path, O_RDONLY);
 //       int64_t size_of_file = file_seek(&fp, 0, SEEK_END);
 //       //printf("The size of file named %s is : %d\n", path, size_of_file);
 //       uint8_t data[80] = {'\0'};
 //       if (fd >= 0)
 //       {
 //         count = address;
 //         do
 //         {
 //           if (size_of_file > 0 && count <= size_of_file)
 //           {
 //             flash.timestamp = (uint64_t)time(NULL);
 //             pkt++;
 //             file_seek(&fp, count, SEEK_SET);
 //             file_read(&fp, data, sizeof(data));
 //             for (int i = 0; i < 80; i++)
 //             {
 //               flash.data[i] = data[i];
 //             }
 //             flash.packet_type = packet_type;
 //             flash.packet_number = pkt;
 //             count += 80;
 
 //             if (OK != orb_publish(ORB_ID(flash_operation), raw_afd, &flash))
 //             {
 //               syslog(LOG_ERR, "Orb Publish failed\n");
 //             }
 //             else
 //             {
 //               syslog(LOG_ERR, "Orb data published\n");
 //             }
 //             num_of_packets -= 1;
 //           }
 //           else
 //           {
 //             break;
 //           }
 
 //           sleep(1);
 //         } while (num_of_packets > 0);
 //       }
 //       file_close(&fp);
 //       struct SEEK_POINTER temp;
 //       fd = file_open(&fp, "/mnt/fs/mfm/mtd_mainstorage/seek_pointer.txt", O_CREAT | O_WRONLY);
 //       if (fd >= 0)
 //       {
 //         if (file_read(&fp, &temp, sizeof(struct SEEK_POINTER)) >= 0)
 //         {
 //           if (strcmp(path, "/mnt/fs/mfm/mtd_mainstorage/satHealth.txt") == 0)
 //             temp.SAT_HEALTH += count;
 //           else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/cam_rgb.txt") == 0)
 //             temp.CAM_RGB += count;
 //           else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/epdm.txt") == 0)
 //             temp.EPDM += count;
 //           else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/cam_nir.txt") == 0)
 //             temp.CAM_NIR += count;
 //           else if (strcmp(path, "/mnt/fs/mfm/mtd_mission/adcs.txt") == 0)
 //             temp.ADCS += count;
 //           if (file_write(&fp, &temp, sizeof(struct SEEK_POINTER)) >= 0)
 //           {
 //             //printf("seek pointer updated\n");
 //           }
 //         }
 //       }
 //     }
 //   }
 // }