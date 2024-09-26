// /****************************************************************************
//  * custom_apps/custom_hello/custom_hello.c
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

// /****************************************************************************
//  * Included Files
//  ****************************************************************************/

// #include <nuttx/config.h>
// #include <stdbool.h>
// #include <stdlib.h>
// #include <stdio.h>
// #include "cushello.h"
// #include "gpio_definitions.h"
// // #include "test2.h"
// uint8_t write_buf = {23, 24, 25, 26};
// static bool g_cushello_daemon_started;
// /****************************************************************************
//  * Public Functions
//  ****************************************************************************/
// typedef struct
// {
//   uint8_t gpio_val;
//   uint32_t gpio_num;
//   // void *data;
// } gpio;

// #ifdef CONFIG_DEBUG_UORB

// static void print_orb_mag_scaled_msg(FAR const struct orb_metadata *meta,
//                                      FAR const void *buffer)
// {
//   FAR const struct orb_mag_scaled_s *message = buffer;
//   const orb_abstime now = orb_absolute_time();

//   uorbinfo_raw("%s:\ttimestamp: %" PRIu64 " (%" PRIu64 " us ago) X: %.4f Y: %.4f Z:%.4f Temp: %.4f",
//                meta->o_name, message->timestamp, now - message->timestamp,
//                message->x, message->y, message->z, message->temperature);
// }
// #endif

// /****************************************************************************
//  * Public Data
//  ****************************************************************************/

// ORB_DEFINE(orb_mag_scaled, struct orb_mag_scaled_s, print_orb_mag_scaled_msg);

// /****************************************************************************
//  * custom_hello_main
//  ****************************************************************************/

// int cushello_daemon(int argc, FAR char *argv[])
// {

//   increment();
//   increment2();
//   g_cushello_daemon_started = true;
//   syslog(LOG_INFO, "Hello World application for writing data to flash.\n");

//   syslog(LOG_INFO, "opening a uORB topic to subscribe messages.\n");

//   struct orb_mag_scaled_s mag_scaled;
//   int instance = 0;
//   bool updated;
//   int afd;

//   struct sensor_mag mag0;
//   struct pollfd fds;
//   int fd;
//   int ret;
//   int i;

//   /* advertie scaled mag topic */

//   afd = orb_advertise_multi_queue_persist(ORB_ID(orb_mag_scaled), &mag_scaled,
//                                           &instance, sizeof(struct orb_mag_scaled_s));
//   if (afd < 0)
//   {
//     syslog(LOG_ERR, "Orb advertise failed.\n");
//   }

//   fd = orb_subscribe_multi(ORB_ID(sensor_mag), 0);

//   fds.fd = fd;
//   fds.events = POLLIN;
//   for (;;)
//   {
//     if (poll(&fds, 1, 3000) > 0)
//     {
//       if (fds.revents & POLLIN)
//       {
//         ret = orb_copy_multi(fd, &mag0, sizeof(struct sensor_mag));
//         if (ret < 0)
//         {
//           syslog(LOG_ERR, "ORB copy error, %d \n", ret);
//           return ret;
//         }

//         // syslog(LOG_INFO, "Copied data from orb_object.\n");

//         // printf("Timestamp: %lli \t", mag0.timestamp);
//         // printf("Temperature: %0.02f \t", mag0.temperature);
//         // printf("X : %0.02f \t", mag0.x);
//         // printf("Y : %0.02f \t", mag0.y);
//         // printf("Z : %0.02f \t\n", mag0.z);
//       }
//       mag_scaled.x = mag0.x * 100;
//       mag_scaled.y = mag0.y * 100;
//       mag_scaled.z = mag0.z * 100;
//       mag_scaled.temperature = mag0.temperature - 50;
//       mag_scaled.timestamp = orb_absolute_time();

//       if (OK != orb_publish(ORB_ID(orb_mag_scaled), afd, &mag_scaled))
//       {
//         syslog(LOG_ERR, "Orb Publish failed\n");
//       }
//     }
//   }

//   ret = orb_unadvertise(afd);
//   if (ret < 0)
//   {
//     syslog(LOG_ERR, "Orb Unadvertise failed.\n");
//   }
//   printf("\n");
//   up_progmem_write(0x081C0000, write_buf, sizeof(write_buf)); // writing data into the internal flash

//   ret = orb_unsubscribe(fd);

//   if (ret < 0)
//   {
//     syslog(LOG_ERR, "Orb unsubscribe Failed.\n");
//   }

//   return 0;
// }

// /****************************************************************************
//  * custom_hello_thread
//  ****************************************************************************/
// int main(int argc, FAR char *argv[])
// {
//   int ret;

//   printf("[Cushello] Starting task.\n");
//   // if (g_cushello_daemon_started)
//   // {
//   //   printf("[Cushello] Task already started.\n");
//   // }

//   // ret = task_create("cushello_daemon", SCHED_PRIORITY_DEFAULT,
//   //                   CONFIG_CUSTOM_APPS_CUSTOM_HELLO_STACKSIZE, cushello_daemon,
//   //                   NULL);

//   // if (ret < 0)
//   // {
//   //   int errcode = errno;
//   //   printf("[cushello] ERROR: Failed to start cushello_dameon: %d\n",
//   //          errcode);
//   //   return EXIT_FAILURE;
//   // }

//   printf("[cushello] cushello_daemon started\n");
//   gpio_write1(GPIO_MUX_EN_EM, false);
//   gpio_write1(GPIO_SFM_CS, false);
//   gpio_write1(GPIO_SFM_MODE, true);
  
//   gpio_write(GPIO_MSN_3V3_EN_EM, true);
//     gpio_write(GPIO_MSN1_EN, true);
//     gpio_write(GPIO_DCDC_MSN_3V3_2_EN, true);
//   return EXIT_SUCCESS;
// }

#include <stdio.h>
#include <math.h>
#include <fcntl.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include "gpio_def.h"

#define DATA_BITS 4
#define PARITY_BITS 3
#define TOTAL_BITS (DATA_BITS + PARITY_BITS)

// Function to calculate parity bits
void calculate_parity_bits(int *data_bits, int *parity_bits) {
    parity_bits[0] = (data_bits[0] + data_bits[1] + data_bits[3]) % 2; // P1
    parity_bits[1] = (data_bits[0] + data_bits[2] + data_bits[3]) % 2; // P2
    parity_bits[2] = (data_bits[1] + data_bits[2] + data_bits[3]) % 2; // P3
}

// Function to detect and correct errors
int detect_and_correct(int *hamming_code) {
    int parity_check[PARITY_BITS];

    parity_check[0] = (hamming_code[0] + hamming_code[2] + hamming_code[4] + hamming_code[6]) % 2; // P1
    parity_check[1] = (hamming_code[1] + hamming_code[2] + hamming_code[5] + hamming_code[6]) % 2; // P2
    parity_check[2] = (hamming_code[3] + hamming_code[4] + hamming_code[5] + hamming_code[6]) % 2; // P3

    // Calculate error position (if any)
    int error_position = parity_check[0] * 1 + parity_check[1] * 2 + parity_check[2] * 4;

    if (error_position != 0) {
        printf("Error detected at position: %d\n", error_position);
        // Flip the erroneous bit (1-based position)
        hamming_code[error_position - 1] ^= 1; // Flip the bit
        return 1; // Error was corrected
    } else {
        printf("No error detected.\n");
        return 0; // No error
    }
}

void print_code(int *code, int size) {
    for (int i = 0; i < size; i++) {
        printf("%d ", code[i]);
    }
    printf("\n");
}

void process_number(int number, int *hamming_code) {
    int data_bits[DATA_BITS];
    int parity_bits[PARITY_BITS];

    // Split the number into 4 bits
    for (int i = 0; i < DATA_BITS; i++) {
        data_bits[DATA_BITS - 1 - i] = (number >> i) & 1; // Extract bits
    }

    // Calculate parity bits
    calculate_parity_bits(data_bits, parity_bits);

    // Construct Hamming code
    hamming_code[0] = parity_bits[0]; // P1
    hamming_code[1] = parity_bits[1]; // P2
    hamming_code[2] = data_bits[0];   // D1
    hamming_code[3] = parity_bits[2]; // P3
    hamming_code[4] = data_bits[1];   // D2
    hamming_code[5] = data_bits[2];   // D3
    hamming_code[6] = data_bits[3];   // D4
}

int restore_number(int *hamming_code) {
    int restored_bits[DATA_BITS];
    restored_bits[0] = hamming_code[2]; // D1
    restored_bits[1] = hamming_code[4]; // D2
    restored_bits[2] = hamming_code[5]; // D3
    restored_bits[3] = hamming_code[6]; // D4

    // Convert restored bits to number
    int number = 0;
    for (int i = 0; i < DATA_BITS; i++) {
        number |= (restored_bits[i] << (DATA_BITS - 1 - i));
    }

    return number;
}

int main(int argc, FAR char *argv[]) {
    int  ret ;
    struct file file_p;
    // char file_path[65];
    char file_path[] = "/mnt/fs/sfm/mtd_mission/camera.txt";
    // sprintf(file_path, "%s/test.txt", mount_point);
    int fd;
    /* //Some problem occured freezing MCU
    if(argc >= 2){
         fd = file_open(&file_p, file_path, O_RDONLY);
        if(fd < 0) 
        {
            char seek_pointer[1000]={'\0'};
            ssize_t bytes_read = file_read(&file_p, seek_pointer, sizeof(seek_pointer));
            printf("Data is %s\n",seek_pointer);
        }
        file_close(&file_p);
        }
        else
        */
        {
    // int number;
    // int hamming_code1[TOTAL_BITS];
    // int hamming_code2[TOTAL_BITS];

    // // Input a number
    // printf("Enter a number (0-255) to process: ");
    // scanf("%d", &number);

    // // Validate the input
    // if (number < 0 || number > 255) {
    //     printf("Please enter a number between 0 and 255.\n");
    //     return 1;
    // }

    // // Split into two 4-bit segments
    // int number1 = (number >> 4) & 0x0F; // High 4 bits
    // int number2 = number & 0x0F;        // Low 4 bits

    // // Process the first 4 bits to compute Hamming code
    // process_number(number1, hamming_code1);
    // // printf("Hamming Code 1: ");
    // print_code(hamming_code1, TOTAL_BITS);

    // // Process the second 4 bits to compute Hamming code
    // process_number(number2, hamming_code2);
    // printf("Hamming Code 2: ");
    // print_code(hamming_code2, TOTAL_BITS);

    // // Simulate an error in the second Hamming code for testing
    // hamming_code2[3] ^= 1; // Flip the 4th bit (for example)
    // printf("Hamming Code 2 with Error: ");
    // print_code(hamming_code2, TOTAL_BITS);

    // // Detect and correct the error in the second Hamming code
    // detect_and_correct(hamming_code2);

    // // Restore numbers from the Hamming codes
    // int restored_number1 = restore_number(hamming_code1);
    // int restored_number2 = restore_number(hamming_code2);

    // // Print the restored numbers
    // printf("Restored Number 1: %d\n", restored_number1);
    // printf("Restored Number 2: %d\n", restored_number2);

    // // Combine both restored numbers into one 8-bit number
    // int combined_number = (restored_number1 << 4) | restored_number2;
    // printf("Combined 8-bit Number: %d\n", combined_number);
    gpio_write(GPIO_MUX_EN, false);
    gpio_write(GPIO_SFM_MODE, false);
    // gpio_write(GPIO_)
    fd = file_open(&file_p, file_path, O_CREAT | O_RDWR | O_APPEND);
    if(fd < 0) 
    {
        syslog(LOG_ERR, "Error opening file in mainstorage of MFM.\n");
        // close(fd);
        file_close(&file_p);
    } else {
        const char *write_data = "Camera, ";
        // ssize_t bytes_written = write(fd, write_data, strlen(write_data));
        ssize_t bytes_written = file_write(&file_p, write_data, strlen(write_data));
        if(bytes_written > 0)
        {
            syslog(LOG_INFO, "Flash Write Successful.\n Data Len: %d\n", bytes_written);
            // close(fd);
            file_close(&file_p);
        } else {
            syslog(LOG_INFO, "Write Failure.\n");
        }
        // int size_file = file_seek(&file_p, 0, SEEK_END);
      
        file_syncfs(&file_p);
        file_close(&file_p);
    gpio_write(GPIO_SFM_MODE, true);
        

    }
    }
    return 0;
}