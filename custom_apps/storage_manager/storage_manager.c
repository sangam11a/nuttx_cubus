#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <mqueue.h>
#include <string.h>
#include <unistd.h>

#define QUEUE_NAME "/gpio"
#define MAX_SIZE    1024
#define MSG_STOP    "exit"

void writer_mq(char *arg1) {
    struct mq_attr attr;
    mqd_t mqd;

    // Initialize attributes
    attr.mq_flags = 0;
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = 8192;
    attr.mq_curmsgs = 0;

    mqd = mq_open(QUEUE_NAME, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR, NULL);
    if (mqd == (mqd_t) -1) {
        perror("mq_open");
        exit(1);
    }

    // Dynamically allocated array of strings
    // char *str[] = {"A", "posix", "message", "queue", "example", "kjsalkfjsdaf","exit"};
    char *str ;
    str = arg1;
    int str_count = sizeof(str) / sizeof(str[0]);

    printf("Writing messages to the POSIX message queue\n\n");
    printf("Number of strings: %d\n", str_count);

    for (int i = 0; i < str_count; i++) {
        // Write to the POSIX message queue
        if (mq_send(mqd, str[i], strlen(str[i]) + 1, 0) == -1) {  // +1 to include the null terminator
            perror("mq_send");
            exit(1);
        }
        printf("Data sent: %s\n", str[i]);
    }

    if (mq_close(mqd) == -1) {
        perror("mq_close");
        exit(1);
    }
}

void reader_mq() {
    struct mq_attr attr;
    mqd_t mqd;

    // Initialize attributes
    attr.mq_flags = 0;
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = 8192;
    attr.mq_curmsgs = 0;

     mqd_t mq;
    char buffer[MAX_SIZE + 1];
    ssize_t bytes_read;

    // Open the message queue
    mq = mq_open(QUEUE_NAME, O_RDONLY);
    if(mq == (mqd_t)-1) {
        perror("mq_open");
        exit(1);
    }

    printf("Waiting for messages...\n");

    // while(1) 
    {
        // Receive the message
        bytes_read = mq_receive(mq, buffer, MAX_SIZE, NULL);
        if(bytes_read == -1) {
            perror("mq_receive");
            exit(1);
        }

        buffer[bytes_read] = '\0';  // Null-terminate the string

        printf("Received: %s\n", buffer);

        // Exit if the received message is "exit"
        
    }

    // Cleanup
    if(mq_close(mq) == -1) {
        perror("mq_close");
        exit(1);
    }

    if(mq_unlink(QUEUE_NAME) == -1) {
        perror("mq_unlink");
        exit(1);
    }

}

int main(int argc, FAR char *argv[]) {

    // writer_mq();
    if (argc > 1){
        if (strcmp(argv[1], "write") == 0x00){
            writer_mq(argv[1]);
        }
        else if(strcmp(argv[1], "read") == 0x00){
            reader_mq();
        }

    }
    else{
            writer_mq(argv[1]);
    }
    return 0;
}
