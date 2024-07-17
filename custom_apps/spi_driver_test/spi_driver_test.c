#include <nuttx/wqueue.h>
#include <nuttx/config.h>
#include <mqueue.h>
#include <nuttx/config.h>
#include <stdio.h>
#include <assert.h>
#include <fcntl.h>

#define QUEUE_NAME "/gpio"
#define MAX_SIZE 1024
#define MSG_STOP "//exit"
#define SEC2TICK(sec) ((sec) * CLK_TCK)
static struct work_s work_gpio1;
static struct work_s work_gpio12;
char *gpio_name_1;
uint8_t pin_mode_1;
static struct work_s work_sec1;

void reader_mq_2(char *gpio_name_1)
{
    struct mq_attr attr;
    mqd_t mqd;

    // Initialize attributes
    attr.mq_flags = 0;
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = 8192;
    attr.mq_curmsgs = 0;

    mqd_t mq;
    char buffer[MAX_SIZE + 1];
    gpio_name_1 = buffer;
    ssize_t bytes_read;

    // Open the message queue
    mq = mq_open(QUEUE_NAME, O_RDONLY);
    if (mq == (mqd_t)-1)
    {
        perror("mq_open");
        // exit(1);
    }

    printf("Waiting for messages...\n");

    // while(1)
    {
        // Receive the message
        bytes_read = mq_receive(mq, buffer, MAX_SIZE, NULL);
        if (bytes_read == -1)
        {
            perror("mq_receive");
            // exit(1);
        }

        buffer[bytes_read] = '\0'; // Null-terminate the string

        printf("Received: %s\n", buffer);
    }

    // Cleanup
    if (mq_close(mq) == -1)
    {
        perror("mq_close");
        // exit(1);
    }

    if (mq_unlink(QUEUE_NAME) == -1)
    {
        perror("mq_unlink");
        // exit(1);
    }
}

void CHECK_GPIO_1()
{
    // while (1)
    {
        printf("called check gpio\n");
        reader_mq_2(gpio_name_1);

        if (!strcmp(gpio_name_1, "COM"))
        {
            printf("Enabling com\n");
            //gpio_write(GPIO_3V3_COM_EN, pin_mode_1);
        }
        else if (!strcmp(gpio_name_1, "MSN1"))
        { // ADCS
            printf("Enabling ADCS\n");

            //gpio_write(GPIO_MSN_3V3_EN, pin_mode_1);
            //gpio_write(GPIO_MSN1_EN, pin_mode_1);
        }
        else if (!strcmp(gpio_name_1, "MSN2"))
        { // CAM
            printf("Enabling CAM\n");

            //gpio_write(GPIO_MSN_3V3_EN, pin_mode_1);
            //gpio_write(GPIO_MSN2_EN, pin_mode_1);
        }
        else if (!strcmp(gpio_name_1, "MSN3"))
        { // EPDM
            //gpio_write(GPIO_MSN_3V3_EN, pin_mode_1);
            //gpio_write(GPIO_MSN3_EN, pin_mode_1);
        }
        else if (!strcmp(gpio_name_1, "MUX"))
        {
            //gpio_write(GPIO_MUX_EN, 1);
            //gpio_write(GPIO_SFM_MODE, pin_mode_1); // TODO: CHECK PULL UP PULL DOWN MODE FOR MSN and OBC access
        }
        else if (!strcmp(gpio_name_1, "ANT"))
        {
            printf("Starting antenna deployment sequence..\n Turning Burner EN line: %d\n Turning Unreg line: %d\n", pin_mode_1, pin_mode_1);
            //gpio_write(GPIO_BURNER_EN, pin_mode_1);
            //gpio_write(GPIO_UNREG_EN, pin_mode_1);
        }
        else
        { // keep on adding other gpio pins as you go
            printf("Unknown command \n");
            // return;
        }
        // usleep(1000000);
        // sleep(2);
    }
    work_queue(HPWORK, &work_gpio12, CHECK_GPIO_1, NULL, SEC2TICK(2));

}


void first()
{
    // while (1)
    {
        printf("first\n");
    }

    int ret = work_queue(HPWORK, &work_gpio1, first, NULL, SEC2TICK(2));
    if (ret < 0)
    {
        printf("Failed to queue work\n");
        // return -1;
    }
}

int main()
{
    int ret;
    // second();
    first();
    CHECK_GPIO_1();
}