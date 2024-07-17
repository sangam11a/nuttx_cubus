#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <mqueue.h>
#include <string.h>

#define QUEUE_NAME  "/test_queue"
#define MAX_SIZE    1024
#define MSG_STOP    "exit"

int main() {
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

    while(1) {
        // Receive the message
        bytes_read = mq_receive(mq, buffer, MAX_SIZE, NULL);
        if(bytes_read == -1) {
            perror("mq_receive");
            exit(1);
        }

        buffer[bytes_read] = '\0';  // Null-terminate the string

        printf("Received: %s\n", buffer);

        // Exit if the received message is "exit"
        if(strcmp(buffer, MSG_STOP) == 0) {
            break;
        }
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

    return 0;
}