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
// void* CPUBenchmarkTask(void *arg);


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


//TODO benchmark testing 
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>

static uint64_t get_time_us(void) {
    struct timeval tc;
    gettimeofday(&tc, NULL);
    return (tc.tv_sec * 1000000LL + tc.tv_usec);
}
void print_heap_info(void)
{
    struct mallinfo info = mallinfo();
    printf("Total space allocated from system: %d bytes\n", info.arena);
    printf("Free space in non-inuse chunks: %d bytes\n", info.fordblks);
    printf("Total allocated space: %d bytes\n", info.uordblks);
}

/* Function to perform 1 KB memory write EPS */
void* MemoryWriteTest(void *arg) {
    struct timeval start, end;
    const uint32_t num_operations = 1000000;  // Number of 1 KiB writes
    uint8_t *buffer = (uint8_t*) malloc(150*1024);  // Allocate 1 KiB buffer
    int fd = open("/dev/ttyS4", O_WRONLY);

    if (fd < 0 || buffer == NULL) {
        perror("Failed to open UART or allocate memory");
        if (buffer) free(buffer);
        return NULL;
    }

    // Get start time
    gettimeofday(&start, NULL);

    // Perform memory writes
    for (uint32_t i = 0; i < num_operations; i++) {
        memset(buffer, 0xAA, 150*1024);  // Write 1 KiB data
    }

    // Get end time
    gettimeofday(&end, NULL);

    // Calculate elapsed time in milliseconds
    long seconds = end.tv_sec - start.tv_sec;
    long microseconds = end.tv_usec - start.tv_usec;
    double elapsed_time_ms = (seconds * 1000.0) + (microseconds / 1000.0);

    // Calculate Events per Second (EPS)
    double events_per_second = (double)num_operations / (elapsed_time_ms / 1000.0);

    // Ensure we do not divide by zero
    if (elapsed_time_ms == 0) {
        events_per_second = 0;
    }

    // Print result via UART
    char buffer_out[100];
    snprintf(buffer_out, sizeof(buffer_out), "Memory Write 1 KB EPS: %f events/sec\n", events_per_second);
    write(fd, buffer_out, strlen(buffer_out));
 print_heap_info();
    // Clean up resources
    free(buffer);
    close(fd);

    return NULL;
}

/* Function to perform random 16 KB read/write EPS */
void* RandomReadWriteTest(void *arg) {
    struct timeval start, end;
    const uint32_t num_operations = 100;  // Number of 16 KiB random read/writes
    uint8_t *buffer = (uint8_t*) malloc(64 * 1024);  // 64 KiB buffer
    int fd = open("/dev/ttyS4", O_WRONLY);

    if (fd < 0 || buffer == NULL) {
        perror("Failed to open UART or allocate memory");
        return NULL;
    }

    // Get start time
    gettimeofday(&start, NULL);

    // Perform random 16 KiB read/writes
    for (uint32_t i = 0; i < num_operations; i++) {
        uint32_t rand_index = (rand() % (64 - 16)) * 1024;  // Random index within the buffer
        memset(buffer + rand_index, 0xAA, 16 * 1024);  // Write 16 KiB data
    }

    // Get end time
    gettimeofday(&end, NULL);

    // Calculate elapsed time in milliseconds
    long seconds = end.tv_sec - start.tv_sec;
    long microseconds = end.tv_usec - start.tv_usec;
    double elapsed_time_ms = (seconds * 1000.0) + (microseconds / 1000.0);

    // Calculate Events per Second (EPS)
    double events_per_second = (double)num_operations / (elapsed_time_ms / 1000.0);

    // Ensure we do not divide by zero
    if (elapsed_time_ms == 0) {
        events_per_second = 0;
    }

    // Print result via UART
    char buffer_out[100];
    snprintf(buffer_out, sizeof(buffer_out), "Random 16 KB R/W EPS: %f events/sec\n", events_per_second);
    write(fd, buffer_out, strlen(buffer_out));

    // Clean up resources
 print_heap_info();

    free(buffer);
    close(fd);

    return NULL;
}

/* Function to perform sequential 16 KB write EPS */
void* SequentialWriteTest(void *arg) {
    struct timeval start, end;
    const uint32_t num_operations = 100;  // Number of 16 KiB sequential writes
    uint8_t *buffer = (uint8_t*) malloc(64 * 1024);  // 64 KiB buffer
    int fd = open("/dev/ttyS4", O_WRONLY);

    if (fd < 0 || buffer == NULL) {
        perror("Failed to open UART or allocate memory");
        return NULL;
    }

    // Get start time
    gettimeofday(&start, NULL);

    // Perform sequential 16 KiB writes
    for (uint32_t i = 0; i < num_operations; i++) {
        memset(buffer + (i % 4) * 16 * 1024, 0xAA, 16 * 1024);  // Write 16 KiB data sequentially
    }

    // Get end time
    gettimeofday(&end, NULL);

    // Calculate elapsed time in milliseconds
    long seconds = end.tv_sec - start.tv_sec;
    long microseconds = end.tv_usec - start.tv_usec;
    double elapsed_time_ms = (seconds * 1000.0) + (microseconds / 1000.0);

    // Calculate Events per Second (EPS)
    double events_per_second = (double)num_operations / (elapsed_time_ms / 1000.0);

    // Ensure we do not divide by zero
    if (elapsed_time_ms == 0) {
        events_per_second = 0;
    }

    // Print result via UART
    char buffer_out[100];
    snprintf(buffer_out, sizeof(buffer_out), "Sequential 16 KB Write EPS: %f events/sec\n", events_per_second);
    write(fd, buffer_out, strlen(buffer_out));

    // Clean up resources
 print_heap_info();

    free(buffer);
    close(fd);

    return NULL;
}

/* Function to perform sequential 16 KB read EPS */
void* SequentialReadTest(void *arg) {
    struct timeval start, end;
    const uint32_t num_operations = 100;  // Number of 16 KiB sequential reads
    uint8_t *buffer = (uint8_t*) malloc(64 * 1024);  // 64 KiB buffer
    int fd = open("/dev/ttyS4", O_WRONLY);

    if (fd < 0 || buffer == NULL) {
        perror("Failed to open UART or allocate memory");
        return NULL;
    }

    // Fill buffer with data for read test
    memset(buffer, 0xAA, 64 * 1024);  // Fill 64 KiB buffer with data

    // Get start time
    gettimeofday(&start, NULL);

    // Perform sequential 16 KiB reads
    for (uint32_t i = 0; i < num_operations; i++) {
        uint8_t temp_buffer[16 * 1024];
        memcpy(temp_buffer, buffer + (i % 4) * 16 * 1024, 16 * 1024);  // Read 16 KiB data sequentially
    }

    // Get end time
    gettimeofday(&end, NULL);

    // Calculate elapsed time in milliseconds
    long seconds = end.tv_sec - start.tv_sec;
    long microseconds = end.tv_usec - start.tv_usec;
    double elapsed_time_ms = (seconds * 1000.0) + (microseconds / 1000.0);

    // Calculate Events per Second (EPS)
    double events_per_second = (double)num_operations / (elapsed_time_ms / 1000.0);

    // Ensure we do not divide by zero
    if (elapsed_time_ms == 0) {
        events_per_second = 0;
    }

    // Print result via UART
    char buffer_out[100];
    snprintf(buffer_out, sizeof(buffer_out), "Sequential 16 KB Read EPS: %f events/sec\n", events_per_second);
    write(fd, buffer_out, strlen(buffer_out));

 print_heap_info();

    // Clean up resources
    free(buffer);
    close(fd);

    return NULL;
}

int main(int argc, char *argv[]) {
    pthread_t thread1, thread2, thread3, thread4;

    // Create the benchmark tasks
    pthread_create(&thread1, NULL, MemoryWriteTest, NULL);
    pthread_create(&thread2, NULL, RandomReadWriteTest, NULL);
    pthread_create(&thread3, NULL, SequentialWriteTest, NULL);
    pthread_create(&thread4, NULL, SequentialReadTest, NULL);

    // Wait for threads to complete (or use pthread_detach if you don't need to join)
    pthread_join(thread1, NULL);
    pthread_join(thread2, NULL);
    pthread_join(thread3, NULL);
    pthread_join(thread4, NULL);

    return 0;
}


// /here 3
// #include <nuttx/config.h>
// #include <stdio.h>
// #include <nuttx/mm/mm.h>
// #include <malloc.h>

// void print_heap_info(void)
// {
//     struct mallinfo info = mallinfo();
//     printf("Total space allocated from system: %d bytes\n", info.arena);
//     printf("Free space in non-inuse chunks: %d bytes\n", info.fordblks);
//     printf("Total allocated space: %d bytes\n", info.uordblks);
// }



// int main(int argc, char *argv[])
// {
//     print_heap_info();
//     return 0;
// }
