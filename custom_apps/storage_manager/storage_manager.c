#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <mqueue.h>
#include <string.h>
#include <unistd.h>

void writer_mq() {
    struct mq_attr attr;
    mqd_t mqd;

    // Initialize attributes
    attr.mq_flags = 0;
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = 8192;
    attr.mq_curmsgs = 0;

    mqd = mq_open("/mqp", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR, NULL);
    if (mqd == (mqd_t) -1) {
        perror("mq_open");
        exit(1);
    }

    // Dynamically allocated array of strings
    char *str[] = {"A", "posix", "message", "queue", "example", "kjsalkfjsdaf"};
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

int main() {
    writer_mq();
    return 0;
}
