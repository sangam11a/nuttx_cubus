
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "gpio_definitions.h"

#define COM_UART "/dev/ttyS0"
#define EPDM_UART "/dev/ttyS2"
#define CAM_UART "/dev/ttyS2"
#define ADCS_UART "/dev/ttyS2"

enum MSNS
{
    COM,
    EPDM,
    ADCS,
    CAM
} MSN_CHOICE;

uint8_t data[7] = {0x53, 0x01, 0x02, 0x03, 0x04, 0x7e};


// int gpio_write(uint32_t pin, uint8_t mode)
// {

//   gpio_config_s gpio_numval;
//   int fd = open(ETX_LED_DRIVER_PATH, O_WRONLY);
//   if (fd < 0)
//   {
//     printf( "Error opening %s for GPIO WRITE...", ETX_LED_DRIVER_PATH);
//     close(fd);
//     return -1;
//   }
//   gpio_numval.gpio_num = pin;
//   gpio_numval.gpio_val = mode;
//   if (gpio_numval.gpio_val > 1 || gpio_numval.gpio_num < 0)
//   {
//     printf( "Undefined GPIO pin or set mode selected...\n");
//     return -2;
//   }
//   int ret = write(fd, (const void *)&gpio_numval, sizeof(gpio_config_s));
//   close(fd);
//   if (ret < 0)
//   {
//     printf("Unable to write to gpio pin...\n");
//   }
//   return ret;
// }

void send_data_uart(char *dev_path, uint8_t *data)
{
    double fd, elapsed = 0, required = 15000 * 1000;
    uint8_t data1[7] = {'\0'};
    int i;
    int count = 0, ret;
    printf("Opening uart dev path : %s ret : %d", dev_path, fd);
    // fd = open(dev_path, O_RDWR);
    // if (fd < 0)
    // {
    //     printf("Unable to open the given uart dev path : %s ret : %d", dev_path, fd);
    // }
    // else
    // {
    // fd = write(fd, data, 7);
    // usleep(10000);
    /*
    ---------------------
    */
    fd = open(dev_path, O_RDWR);
    if (fd < 0)
    {
        printf("error opening %s\n", CAM_UART);
        return -1;
    }

    int wr1 = write(fd, data, 7);
    printf("\n%d bytes written\n", wr1);
    do
    {
        printf("Elapsed: %d\n", elapsed);
        usleep(10000);
        for (i = 0; i < 7; i++)
        {
            ret = read(fd, &data1[i], 1);
        }

        // if (ret > 0)
        {
            printf("data received from %s fd value: %d\n", dev_path, fd);
            // printf(data1);
            for (i = 0; i < 7; i++)
            {
                printf(" %x ", data1[i]);
                printf(" %c ", (char *)data1[i]);
            }
            // break;
            if (data[0] == data1[0] && data[1] == data1[1] && data[2] == data1[2] && data[3] == data1[3] && data[4] == data1[4] && data[5] == data1[5])
            {
                printf("\n******Acknowledgement received\n");
                break;
            }
            break;
            // return;
        }
        elapsed += 10000;
        if (elapsed > required)
            break;
    } while (ret < 6);
    printf("handshake complete\n");
    uint8_t send_data[84]; //={0x53,0x01,0x54};
    send_data[0] = 0x53;
    send_data[1] = 0x01;
    send_data[2] = 0x54;
    for (int i = 3; i < 83; i++)
    {
        send_data[i] = i;
    }
    send_data[83] = 0x7e;
    close(fd);
    usleep(10000);
    fd = open(COM_UART, O_WRONLY);

    if (fd < 0)
    {
        printf("unable to open file \n");
        return -2;
    }
    gpio_write(GPIO_COM_4V_EN, 1);
    ret = write(fd, send_data, 84);
    usleep(10000);
    if(ret < 0){
        printf("unable to send data\n ret value: %d\n", ret);
        for(i=0;i<84;i++){
            ret = write(fd, &send_data[i], 1);
            usleep(1000);
        }
    }
    usleep(1000 * 1000 *  5);
    gpio_write(GPIO_COM_4V_EN, 0);
    printf("value of ret: %d\n", ret);
    close(fd);
    printf("Com app finished\n");

    // }
    /*
    ---------------------
    */
    // if(fd > 0){
    // while (elapsed < required)
    // {
    //     // receive_data_uart(dev_path, data1);
    //     ret = read(fd, data1, sizeof(data1));
    //     usleep(15000);
    //     if (ret > 0)
    //     {
    //         printf("data received from CAM\n");
    //         if (data[0] == data1[0] && data[1] == data1[1] && data[2] == data1[2] && data[3] == data1[3] && data[4] == data1[4] && data[5] == data1[5])
    //         {
    //             printf("\n******Acknowledgement received for %s*******\n", dev_path);
    //             break;
    //         }
    //         // return;
    //     }

    //     elapsed += 1000;
    //     usleep(1000);
    //     // printf("delay timeout...\n elapsed: %d\n", elapsed);
    // }
    // }
    // }
    printf("Out of the function....\n");
    // close(fd);
}

void receive_data_uart(char *dev_path, uint8_t *data)
{
    int fd;
    fd = open(dev_path, O_RDWR);
    char var[100];
    // if (fd < 0)
    // {
    //     // syslog(LOG_ERROR, "Unable to open the given uart dev path : %s ret : %d",dev_path, fd);
    // }
    // else
    // {
        fd = read(fd, data, sizeof(data));
        if (fd > 0)
        {
            printf("data received from CAM\n");
            // return;
        }
        // syslog(LOG_//syslog,"Data written successfully");
    // }
    close(fd);
}

void send_data_uart_char(char *dev_path, uint8_t *data)
{
    int fd;
    fd = open(dev_path, O_RDWR);
    if (fd < 0)
    {
        // syslog(LOG_ERROR, "Unable to open the given uart dev path : %s ret : %d",dev_path, fd);
    }
    else
    {
        fd = write(fd, data, sizeof(data));
        if (fd > 0)
        {
            printf("\n");
            // syslog(LOG_//syslog,"Data written successfully");
        }
    }
    close(fd);
}

// void send_data

/*
Beacon type 1 is of 84 including header+packet_type+ data + information/data + footer
*/
void beacon_type_1()
{
    int fd = open(COM_UART, O_RDWR);
    if(fd<0){
        printf("unable to open uart\n");
    }
    uint8_t data[84]; //={0x53,0x01,0x54};
    data[0] = 0x53;
    data[1] = 0x01;
    data[2] = 0x54;
    for (int i = 3; i < 83; i++)
    {
        data[i] = i;
    }
    data[83] = 0x73;
    int ret = write(COM_UART, data, 84);
    if(ret < 0){
        printf("unable to send data\n");
    }
    usleep(50000);
    close(fd);
}

/*
Beacon type 2 is of 84 including header+packet_type+ data + information/data + footer
*/
void beacon_type_2()
{
    uint8_t data[85]; //={0x53,0x01,0x54};
    data[0] = 0x53;
    data[1] = 0x02;
    data[2] = 0x54;
    for (int i = 3; i < 83; i++)
    {
        data[i] = i;
    }
    data[83] = 0x73;
    data[84] = "\0";
    write(COM_UART, data, sizeof(data));
}

/*
Receive telecommand from GS for 90 seconds
*/
void receive_telecommand_rx()
{
    uint8_t data[255];
    receive_data_uart(COM_UART, data);
}
/*
COM will be in digipeater mode till a digipeating message is received and digipeated
*/
void digipeater_mode(uint8_t *data)
{
    receive_data_uart(COM_UART, data);
}

void myPrintf(uint8_t *data)
{
    printf("The data is :\n");
    for (int i = 0; i < sizeof(data); i++)
    {
        printf("%d", data[i]);
    }
    printf("\n");
}

/*
Handshake command is to be provided 6bytes with a header 0x53 and footer 0x7e
*/
void handshake(int choice, uint8_t ack[])
{
    float counter = 0, expected = 90000 * 1000;

    uint8_t data[100];
    printf("handshake fun called choice is %d\n", choice);

    switch (choice)
    {
    case COM:
        printf("COM uart");
        send_data_uart(COM_UART, ack);// handshaking
        usleep(1000 * 2000);
        // gpio_write(GPIO_COM_4V_EN, 1);
        // beacon_type_1(); //sending beacon 1
        usleep(1000 * 1000 * 6);
        // gpio_write(GPIO_COM_4V_EN, 0);
        // while (counter < expected)
        // {
        //     receive_data_uart(COM_UART, data);
        //     printf("COM uart here %d\n", counter);
        //     usleep(1000);
        //     counter += 1000;
        // }
        myPrintf(data);
        // beacon_type_2();
        memset(data, '\0', sizeof(data));
        // do
        // {
        //     digipeater_mode(data);

        // } while (data[0] != 0 & data[1] != 0);
        myPrintf(data);
        break;
    case EPDM:
        printf("COM epdm");
        send_data_uart(EPDM_UART, ack);
        break;
    case ADCS:
        printf("ADCS uart");
        send_data_uart(ADCS_UART, ack);
        break;
    case CAM:
        printf("CAM uart");
        send_data_uart(CAM_UART, ack);
        break;
    default:
        send_data_uart_char("/dev/ttyS4", "\nCaught on default\n");
        break;
    };
}

/****************************************************************************
 * custom_hello_thread
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
    // printf("Custom com cmd %d", argc);
    if (argc > 1)
    {
        printf("\n");
        if (strcmp(argv[1], "com") == 0x00)
        {
            int retval = task_create("task1", 100, 528, COM_TASK, NULL);
        }
        else if (strcmp(argv[1], "epdm") == 0x00)
        {
            handshake(EPDM, data);
        }
        else if (strcmp(argv[1], "cam") == 0x00)
        {
            handshake(CAM, data);
        }
        else if (strcmp(argv[1], "adcs") == 0x00)
        {
            handshake(ADCS, data);
        }
        else
        {
            handshake(77, data);
        }
    }
    else
    {
        int i, ret, fd;
        uint8_t data1[7];
        fd = open(CAM_UART, O_RDWR);
        if (fd < 0)
        {
            printf("error opening %s\n", CAM_UART);
            return -1;
        }

        int wr1 = write(fd, data, 6);
        printf("%d bytes written\n", wr1);
        do
        {
            for (i = 0; i < 7; i++)
            {
                ret = read(fd, &data1[i], 1);
            }

            if (ret > 0)
            {
                printf("%d data received from CAM: fd value: %d\n", ret, fd);
                // printf(data1);
                for (i = 0; i < 7; i++)
                {
                    printf(" %x ", data1[i]);
                    printf(" %c ", (char *)data1[i]);
                }
                // break;
                if (data[0] == data1[0] && data[1] == data1[1] && data[2] == data1[2] && data[3] == data1[3] && data[4] == data1[4] && data[5] == data1[5])
                {
                    printf("\n******Acknowledgement received\n");
                }
                break;
                // return;
            }
        } while (ret < 6);

        close(fd);
        printf("Com app finished\n");
    }

    return 0;
}

static int COM_TASK(int argc, char *argv[]){
    handshake(COM, data);
  for(;;){
    
  }
}