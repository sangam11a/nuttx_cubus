// /****************************************************************************
//  * custom_apps/spi_adc_test/spi_test_main.c
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
// #include <stdio.h>
// #include <assert.h>
// #include <fcntl.h>
// #include "cubus_mtd.h"

// #include <sys/mount.h>
// #include <sys/stat.h>
// #include <sys/statfs.h>

// /****************************************************************************
//  * Public Functions
//  ****************************************************************************/
// // struct 

// /****************************************************************************
//  * spi_test_main
//  ****************************************************************************/
// static int task2(int argc, char *argv[]);
// static int task1(int argc, char *argv[]);

// int main(int argc, FAR char *argv[])
// {
//   printf("Multi-tasking application....\n");
//   // cubus_mft_configure(board_get_manifest());
//   struct file file1;
  
//   // char file_name[][4] = {"/mnt/fs/sfm/mtd_mainstorage/test.txt\0","/mnt/fs/sfm/mtd_mainstorage/satHealth.txt\0","/mnt/fs/sfm/mtd_mainstorage/flags.txt\0","/mnt/fs/sfm/mtd_mainstorage/seek.txt\0"};
//   int fd ;
//   int ret;
//   // for(int i=0;i<4;i++)
//     struct file fp;
//   // int fd = open_file_flash(&fp, filepath, filename, O_RDONLY);
//   // if(fd < 0){
//   //   return -1;
//   // }
//   {
//     char buffer[300];
//   fd = file_open(&file1, "/mnt/fs/sfm/mtd_mainstorage/satelliteHealth.txt", O_CREAT|O_RDONLY);
//   // int ret;
//   int file_size = file_seek(&file1,0, SEEK_END);
//   file_seek(&file1,file_size-40, SEEK_END);
//   ret = file_read(&file1,buffer, file_size);//file_write(&file1, file_name[i], strlen(file_name[i]));

//   printf("Size of file is %d\n Data is %s\n", file_size, buffer);
//   file_close(&file1);
//   fd = file_open(&file1, "/mnt/fs/sfm/mtd_mainstorage/test.txt", O_CREAT|O_RDONLY);
//    file_size = file_seek(&file1, 0, SEEK_END);
//   ret = file_read(&file1, buffer, file_size);//file_write(&file1, file_name[i], strlen(file_name[i]));

//   printf("Size of file is %d\n Data is %s\n", file_size, buffer);
//   file_close(&file1);
//   }
   

//   // int ret = 0;
//   // ret = task_create("task1", 100, 528, task1, NULL);
//   // if(ret < 0){
//   //   printf("Error starting task 1\n");
//   // }else{
//   //   printf("task 1 started...\n");
//   // }
//   // ret = task_create("task2", 1, 528, task2, NULL);
//   // if(ret < 0){
//   //   printf("Error starting task 2\n");
//   // }else{
//   //   printf("task 2 started...\n");
//   // }
// }

// static int task1(int argc, char *argv[]){
//   for(;;){
//     printf("Inside Task 1 ... \n");
//     usleep(1000*1000);
//     printf("After delay...task 1\n");
//     usleep(1000 * 1000);
//   }
// }

// static int task2(int argc, char *argv[]){
//   for(;;){
//    printf("Inside Task 2 ... \n");
//    usleep(1000*1000);
//    printf("After delay...task 2\n");
//     usleep(1000 * 100);
//   }
// }

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#define BUFFER_SIZE 81 // 30 bytes + 1 for null terminator

void read_last_30_bytes(const char *file_path) {
    char buffer[BUFFER_SIZE];
    int fd, file_size, ret;

    // Open the file in read-only mode
    fd = open(file_path, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open file");
        return;
    }

    // Get the size of the file
    file_size = lseek(fd, 0, SEEK_END);
    if (file_size < 0) {
        perror("Failed to get file size");
        close(fd);
        return;
    }

    // Seek to the position 30 bytes before the end of the file
    int seek_position = (file_size > BUFFER_SIZE-1) ? file_size - BUFFER_SIZE-1 : 0;
    if (lseek(fd, seek_position, SEEK_SET) < 0) {
        perror("Failed to seek in file");
        close(fd);
        return;
    }

    // Read the last 30 bytes (or less if file size is less than 30)
    ret = read(fd, buffer, (file_size > BUFFER_SIZE-1) ? BUFFER_SIZE-1 : file_size);
    if (ret < 0) {
        perror("Failed to read file");
        close(fd);
        return;
    }

    // Null-terminate the buffer
    buffer[ret] = '\0';

    // Print the results
    printf("Size of file is %d\nData is: %s\n", file_size, buffer);

    // Close the file
    close(fd);
}

int main() {
    read_last_30_bytes("/mnt/fs/sfm/mtd_mainstorage/satelliteHealth.txt");
    read_last_30_bytes("/mnt/fs/sfm/mtd_mainstorage/test.txt");
    return 0;
}
