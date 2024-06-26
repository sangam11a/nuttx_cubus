#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <stdint.h>

#include <fcntl.h>
#include <sys/ioctl.h>


enum MSNS {
COM,
EPDM,
ADCS,
CAM
} MSN_CHOICE;



void send_data_uart(char *dev_path, int *data){
    int fd;
    fd = open(dev_path, O_RDWR);
    if(fd < 0){
        //syslog(LOG_ERROR, "Unable to open the given uart dev path : %s ret : %d",dev_path, fd);
    }
    else{
        fd = write(fd, data, sizeof(data));
        if(fd > 0){
            //syslog(LOG_//syslog,"Data written successfully");
        }
    }
    close(fd);
}   

/*
Handshake command is to be provided 6bytes with a header 0x53 and footer 0x7e
*/
void handshake(int choice, uint8_t ack[7]){
    switch(choice){
        case COM:
                break;
        case EPDM: 
                break;
        case ADCS: 
                break;
        case CAM: 
                break;
        default: 
                break;
    };
}

/*
Beacon type 1 is of 84 including header+packet_type+ data + information/data + footer
*/
void beacon_type_1(){

}

/*
Beacon type 2 is of 84 including header+packet_type+ data + information/data + footer
*/
void beacon_type_2(){

}


/*
Receive telecommand from GS for 90 seconds
*/
void receive_telecommand_rx(){

}
/*
COM will be in digipeater mode till a digipeating message is received and digipeated
*/
void digipeater_mode(){

}

/****************************************************************************
 * custom_hello_thread
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
    return 0;
}
