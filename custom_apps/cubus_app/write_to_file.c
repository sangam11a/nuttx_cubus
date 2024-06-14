// #include <sys/mount.h>
// #include <sys/stat.h>
// #include <sys/statfs.h>

// #include <stdio.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <string.h>
// #include <fcntl.h>
// #include <dirent.h>
// #include <errno.h>


// int write_to_file(char *MOUNT1,char *filename, char DATA[255]){
//   int fd, ret;
//   char file[255]="\0";
//   // strcat(MOUNT1,filename);
//   snprintf(file,sizeof(file), "%s%s",MOUNT1, filename);
//   printf("MOUNT1 filename file %s %s %s\n",MOUNT1,filename, file);
//   fd = open(file , O_CREAT | O_APPEND | O_WRONLY, 0755);
//     if (fd < 0) {
//         printf("Failed to open file: %s\n", strerror(errno));
//     } else {
//         printf("File has been opened with name numbers.txt\n");
//         // char DATA[] = "Testing the writing to file works or not.\n";

//         ret = write(fd, DATA, strlen(DATA));
//         if (ret < 0) {
//             printf("Failed to write the data: %s\n", strerror(errno));
//             close(fd);
//             return 1;
//         } else {
//             printf("Data written successfully\n");
//         }
//         close(fd);
//     }
  
// }