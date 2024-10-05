#include <nuttx/wqueue.h>
#include <nuttx/config.h>
#include <mqueue.h>
#include <nuttx/config.h>
#include <stdio.h>
#include <assert.h>
#include "gpio_control.h"
#include <fcntl.h>
#include <debug.h>
#include <sys/ioctl.h>

#define QUEUE_NAME "/gpio"
#define MAX_SIZE 1024
#define MSG_STOP "//exit"
static struct work_s work_gpio1;
static struct work_s work_gpio12;
char *gpio_name_1;
uint8_t pin_mode_1;
static struct work_s work_sec1;
uint8_t data[7] = {0x53, 0x01, 0x02, 0x03, 0x04, 0x7e};

typedef struct __attribute__((__packed__)) _BEACON_A
{
  uint8_t HEAD;     // 1 byte head -0x53
  uint8_t TYPE : 4; // 4bit
  int TIM_DAY : 12; // 12bit
  uint8_t TIM_HOUR; // 1byte

  uint16_t BAT_V;     // voltage 2 byte
  uint16_t BAT_C;     // current 2 byte
  int16_t BAT_T;      // battery temperatuerein Degree centigrade 2byte
  int8_t RAW_C;       // 1byte
  uint16_t SOL_TOT_V; // in mV  2 byte
  int16_t SOL_TOT_C;  // in mA 2 byte

  int8_t BPB_T; // backplane board temp 1byte
  int8_t OBC_T; // 1 byte
  /*TOdo make consist var*/
  int8_t Y1_T; // 1 byte
  int8_t Y_T;  // 1 byte
  int8_t Z1_T; // 1 byte
  int8_t Z_T;  // 1 byte
  int8_t X1_T; // 1 byte
  int8_t X_T;  // 1 byte

  uint8_t SOL_P1_STAT; // 1 byte
  uint8_t SOL_P2_STAT; // 1 byte
  uint8_t SOL_P3_STAT; // 1 byte
  uint8_t SOL_P4_STAT; // 1 byte
  uint8_t MSN1_STAT;   // 1 byte ()
  uint8_t MSN2_STAT;   // 1 byte ()
  uint8_t MSN3_STAT;   // 1 byte ()

  uint8_t ANT_STAT;   // 1 byte
  uint8_t KILL1_STAT; // 1 byte
  uint8_t KILL2_STAT; // 1 byte
  uint8_t UL_STAT;    // 1 byte

  uint8_t OPER_MODE; // 1 byte
  uint16_t RST_RESET_COUNT;
  uint16_t OBC_RESET_COUNT; // 2 byte
  uint16_t LAST_RESET;      // 2 byte
  uint16_t CHK_CRC;         // 2 byte

  uint8_t ANT_P_T;

  // uint16_t
} S2S_BEACON_A;

typedef struct __attribute__((__packed__)) _S2S_BEACON_TYPE_B
{
  /*1-byte*/ uint8_t HEAD;
  /*4-bit*/ uint8_t TYPE : 4;
  /*12-bit*/ unsigned int TIM_DAY : 12;

  /*1-byte*/ uint8_t SOL_P1_V; // voltage upto 1dp, without decimal
  /*1-byte*/ uint8_t SOL_P2_V;
  /*1-byte*/ uint8_t SOL_P3_V;
  /*1-byte*/ uint8_t SOL_P4_V;

  /*1-byte*/ int8_t SOL_P1_C;
  /*1-byte*/ int8_t SOL_P2_C;
  /*1-byte*/ int8_t SOL_P3_C;
  /*1-byte*/ int8_t SOL_P4_C;

  /*2-byte*/ int16_t GYRO_X;
  /*2-byte*/ int16_t GYRO_Y;
  /*2-byte*/ int16_t GYRO_Z;
  /*2-byte*/ int16_t ACCL_X;
  /*2-byte*/ int16_t ACCL_Y;
  /*2-byte*/ int16_t ACCL_Z;

  /*2-byte*/ int16_t MAG_X;
  /*2-byte*/ int16_t MAG_Y;
  /*2-byte*/ int16_t MAG_Z;
  /*1-byte*/ uint8_t CHK_CRC;
} S2S_BEACON_TYPE_B; // Use typedef for ease of use

S2S_BEACON_A s2s_beacon_type_a;
S2S_BEACON_TYPE_B s2s_beacon_type_b;
// uint8_t Msn_Start_Cmd[7] = {0x53, 0xad, 0xba, 0xcd, 0x9e, 0x7e};
// uint8_t ACK[7] = {0x53, 0xaa, 0xcc, 0xaa, 0xcc, 0x7e};
uint8_t NACK[7] = {0x53, 0xee, 0xff, 0xee, 0xff, 0x7e};

// uint8_t RX_DATA_EPDM[48] = {'\0'};

uint8_t digipeating = 1;

#define PRINT_DELAY 500

#define COM_UART "/dev/ttyS0"
#define EPDM_UART "/dev/ttyS5"
#define BEACON_DELAY 90
#define BEACON_DATA_SIZE 85
#define ACK_DATA_SIZE 6 + 1
#define COM_RX_CMD_SIZE 29
#define COM_DG_MSG_SIZE 30
uint8_t beacon_status = 0;

uint8_t COM_BUSY = 0;
static struct work_s work_beacon;
uint8_t beacon_type = 0;

// TODO checking the seek pointer
void retrieve_data_from_flash_edited(char *partition_name, char *filename, uint8_t *data_retrieved, int offset_value)
{
  // struct stat st;
  struct file fptr;
  struct file fp;
  int fd = 0;
  char path[80];
  sprintf(path, "%s%s", partition_name, filename); // save_data_to_flash("/mnt/fs/mfm/mtd_mission", "/cam.txt", &data1)
  fd = file_open(&fp, path, O_CREAT | O_RDONLY);
  if (fd >= 0)
  {
    // int offset_value = file_seek(&fptr, 0, SEEK_END);
    int off = file_seek(&fptr, offset_value, SEEK_SET);
    ssize_t bytes_read = file_read(&fp, data_retrieved, sizeof(data_retrieved));
    if (bytes_read > 0)
    {
      print_retrieved_data(data_retrieved, bytes_read);
    }
    file_syncfs(&fp);
    if (file_close(&fp))
    {
      // fd = file_open(&fp, path, O_WRONLY);
      // {
      //   if(fd > 0){
      //     ssize_t bytes_write = file_write(&fp, );
      //   }
      // }
    }
    close(fd);
  }
  else
  {
    close(fd);

    syslog(LOG_ERR, "Error opening file to read satellite health data..\n");
  }
  file_close(&fptr);
}

// int open_file_flash(struct file *file_pointer, char *flash_strpath, char *filename, int open_mode){

//   const char file_name[] = {'\0'};
//   // memcpy(file_name, filename, sizeof(filename));
//   char path[65];
//   sprintf(path, "%s%s", flash_strpath, filename);
//   int fd = file_open(file_pointer, path, open_mode);
//   if(fd < 0){
//      syslog(LOG_ERR, "Error opening file: %s\n",path);
//      return fd;
//   }else{
//     syslog(LOG_INFO, "Opened file: %s ...\n",path);
//   }
//   return fd;
// }

void save_data_to_flash(char *partition_name, char *filename, uint8_t *data_retrieved)
{
  struct file file_p;

  int fd = 0;
  fd = file_open(&file_p, "/mnt/fs/mfm/mtd_mission/cam.txt", O_CREAT | O_WRONLY | O_APPEND);
  printf("value of fd is %d\n", fd);
  // fd = open_file_flash(&fptr, partition_name, filename, O_RDONLY | O_CREAT);
  if (fd >= 0)
  {
    ssize_t bytes_written = file_write(&file_p, data_retrieved, sizeof(data_retrieved));
    if (bytes_written > 0)
    {
      syslog(LOG_INFO, "Camera data write Successful.\nData Len: %d.\n", bytes_written);
      file_close(&file_p);
    }
    else
    {
      syslog(LOG_INFO, "Write Failure.\n");
    }
    file_syncfs(&file_p);
    file_close(&file_p);
  }
  else
  {
    syslog(LOG_ERR, "Error opening file to camera.txt..\n");
  }
  file_close(&file_p);
}

/****************************************************************************
 * Receive data from UART
 ****************************************************************************/
int receive_data_uart(char *dev_path, uint8_t *data, uint16_t size)
{
  int fd, ret;
  fd = open(dev_path, O_RDONLY);
  // ioctl(fd, TCFLSH, 2);    //check if we can receive data without flushing initially
  // ioctl(fd, TCDRN, NULL);
  // printf("drained and flushed tx rx buffer\n");
  if (fd < 0)
  {
    printf("Unable to open %s\n", dev_path);
    return fd;
  }

  printf("size of data to receive: %d\n", size);
  // ret = read(fd, data, sizeof(data)); //try receiving data without byte by byte method
  for (int i = 0; i < size; i++)
  {
    ret = read(fd, &data[i], 1);
  }
  if (ret < 0)
  {
    printf("data Not received from %s\n", dev_path);
    return ret;
  }
  printf("Data received from %s\n", dev_path);
  for (int i = 0; i < size; i++)
  {
    printf("%x ", data[i]);
  }
  printf("\n");
  ioctl(fd, TCFLSH, 2);
  ioctl(fd, TCDRN, NULL);
  printf("drained and flushed tx rx buffer\n");
  close(fd);
  return ret;
}
/****************************************************************************
 * Send data from UART through any UART path
 ****************************************************************************/
int send_data_uart(char *dev_path, uint8_t *data, uint16_t size)
{
  double fd;
  int i;
  int count = 0, ret;
  printf("Opening uart dev path : %s\n", dev_path);
  fd = open(dev_path, O_WRONLY);

  if (fd < 0)
  {
    printf("error opening %s\n", dev_path);
    return fd;
  }
  int wr1 = write(fd, data, size);
  if (wr1 < 0)
  {
    printf("Unable to write data\n");
    return wr1;
  }
  printf("\n%d bytes written\n", wr1);
  ioctl(fd, TCFLSH, 2);
  printf("flused tx rx buffer\n");
  ioctl(fd, TCDRN, NULL);
  printf("drained tx rx buffer\n");
  close(fd);
  return wr1;
}

void serialize_beacon_a(uint8_t beacon_data[BEACON_DATA_SIZE])
{
  for (int i = 0; i <= BEACON_DATA_SIZE; i++)
  {
    beacon_data[83] = 0x00;
    // }
    // beacon_data[84] = s2s_beacon_type_a.;
    // beacon_data[85] = s2s_beacon_type_a.;
  }
  // uint8_t beacon_data[BEACON_DATA_SIZE];
  beacon_data[0] = s2s_beacon_type_a.HEAD;
  beacon_data[1] = s2s_beacon_type_a.TYPE;
  beacon_data[2] = s2s_beacon_type_a.TIM_DAY;
  beacon_data[3] = s2s_beacon_type_a.TIM_HOUR;
  beacon_data[4] = (s2s_beacon_type_a.BAT_V >> 8) & 0Xff;
  beacon_data[5] = s2s_beacon_type_a.BAT_V & 0xff;
  beacon_data[6] = (s2s_beacon_type_a.BAT_C >> 8) & 0Xff;
  beacon_data[7] = (s2s_beacon_type_a.BAT_V) & 0Xff;
  beacon_data[8] = (s2s_beacon_type_a.BAT_T >> 8) & 0Xff;
  beacon_data[9] = (s2s_beacon_type_a.BAT_T) & 0Xff;
  beacon_data[10] = s2s_beacon_type_a.RAW_C;
  beacon_data[11] = (s2s_beacon_type_a.SOL_TOT_V >> 8) & 0Xff;
  beacon_data[12] = (s2s_beacon_type_a.SOL_TOT_V) & 0Xff;
  beacon_data[13] = (s2s_beacon_type_a.SOL_TOT_C >> 8) & 0Xff;
  beacon_data[14] = (s2s_beacon_type_a.SOL_TOT_C >> 8) & 0Xff;
  beacon_data[15] = s2s_beacon_type_a.ANT_P_T;
  beacon_data[16] = s2s_beacon_type_a.BPB_T;
  beacon_data[17] = s2s_beacon_type_a.OBC_T;
  beacon_data[18] = s2s_beacon_type_a.X_T;
  beacon_data[19] = s2s_beacon_type_a.X1_T;
  beacon_data[20] = s2s_beacon_type_a.Y_T;
  beacon_data[21] = s2s_beacon_type_a.Y1_T;
  // beacon_data[] = s2s_beacon_type_a.SOL_P5_T;
  beacon_data[22] = s2s_beacon_type_a.SOL_P1_STAT << 7 & s2s_beacon_type_a.SOL_P2_STAT << 6 & s2s_beacon_type_a.SOL_P3_STAT << 5 & s2s_beacon_type_a.SOL_P4_STAT << 4 & s2s_beacon_type_a.MSN1_STAT << 3 & s2s_beacon_type_a.MSN2_STAT << 2 & s2s_beacon_type_a.MSN3_STAT << 1 & 0xff;
  beacon_data[23] = s2s_beacon_type_a.ANT_STAT << 4 & s2s_beacon_type_a.UL_STAT << 4;
  beacon_data[24] = s2s_beacon_type_a.OPER_MODE;
  beacon_data[25] = (s2s_beacon_type_a.OBC_RESET_COUNT >> 8) & 0xff;
  beacon_data[26] = s2s_beacon_type_a.OBC_RESET_COUNT & 0xff;
  beacon_data[27] = s2s_beacon_type_a.RST_RESET_COUNT >> 8 & 0xff;
  beacon_data[28] = s2s_beacon_type_a.RST_RESET_COUNT & 0xff;
  beacon_data[29] = s2s_beacon_type_a.LAST_RESET;
  beacon_data[30] = s2s_beacon_type_a.CHK_CRC;
  // beacon_data[31] = s2s_beacon_type_a.;
  // beacon_data[32] = s2s_beacon_type_a.;
  // beacon_data[34] = s2s_beacon_type_a.;
  // beacon_data[35] = s2s_beacon_type_a.;
  // beacon_data[36] = s2s_beacon_type_a.;
  // beacon_data[37] = s2s_beacon_type_a.;
  // beacon_data[38] = s2s_beacon_type_a.;
  // beacon_data[39] = s2s_beacon_type_a.;
  // beacon_data[40] = s2s_beacon_type_a.;
  // beacon_data[41] = s2s_beacon_type_a.;
  // beacon_data[42] = s2s_beacon_type_a.;
  // beacon_data[43] = s2s_beacon_type_a.;
  // beacon_data[44] = s2s_beacon_type_a.;
  // beacon_data[45] = s2s_beacon_type_a.;
  // beacon_data[46] = s2s_beacon_type_a.;
  // beacon_data[47] = s2s_beacon_type_a.;
  // beacon_data[48] = s2s_beacon_type_a.;
  // beacon_data[49] = s2s_beacon_type_a.;
  // beacon_data[50] = s2s_beacon_type_a.;
  // beacon_data[51] = s2s_beacon_type_a.;
  // beacon_data[52] = s2s_beacon_type_a.;
  // beacon_data[53] = s2s_beacon_type_a.;
  // beacon_data[54] = s2s_beacon_type_a.;
  // beacon_data[55] = s2s_beacon_type_a.;
  // beacon_data[56] = s2s_beacon_type_a.;
  // beacon_data[57] = s2s_beacon_type_a.;
  // beacon_data[58] = s2s_beacon_type_a.;
  // beacon_data[59] = s2s_beacon_type_a.;
  // beacon_data[60] = s2s_beacon_type_a.;
  // beacon_data[61] = s2s_beacon_type_a.;
  // beacon_data[62] = s2s_beacon_type_a.;
  // beacon_data[63] = s2s_beacon_type_a.;
  // beacon_data[64] = s2s_beacon_type_a.;
  // beacon_data[65] = s2s_beacon_type_a.;
  // beacon_data[66] = s2s_beacon_type_a.;
  // beacon_data[67] = s2s_beacon_type_a.;
  // beacon_data[68] = s2s_beacon_type_a.;
  // beacon_data[69] = s2s_beacon_type_a.;
  // beacon_data[70] = s2s_beacon_type_a.;
  // beacon_data[71] = s2s_beacon_type_a.;
  // beacon_data[72] = s2s_beacon_type_a.;
  // beacon_data[73] = s2s_beacon_type_a.;
  // beacon_data[74] = s2s_beacon_type_a.;
  // beacon_data[75] = s2s_beacon_type_a.;
  // beacon_data[76] = s2s_beacon_type_a.;
  // beacon_data[77] = s2s_beacon_type_a.;
  // beacon_data[78] = s2s_beacon_type_a.;
  // beacon_data[79] = s2s_beacon_type_a.;
  // beacon_data[80] = s2s_beacon_type_a.;
  // beacon_data[81] = s2s_beacon_type_a.;
  // beacon_data[82] = s2s_beacon_type_a.;
  // beacon_data[83] = s2s_beacon_type_a.;
}
void serialize_beacon_b(uint8_t beacon_data[BEACON_DATA_SIZE])
{
  for (int i = 0; i <= BEACON_DATA_SIZE; i++)
  {
    beacon_data[83] = 0x00;
  }
  // uint8_t beacon_data[BEACON_DATA_SIZE];
  beacon_data[0] = s2s_beacon_type_b.HEAD;
  beacon_data[1] = s2s_beacon_type_b.TYPE;
  beacon_data[2] = s2s_beacon_type_b.TIM_DAY;
  beacon_data[3] = s2s_beacon_type_b.TIM_DAY;

  beacon_data[4] = s2s_beacon_type_b.SOL_P1_V;
  beacon_data[5] = s2s_beacon_type_b.SOL_P2_V;
  beacon_data[6] = s2s_beacon_type_b.SOL_P3_V;
  beacon_data[7] = s2s_beacon_type_b.SOL_P4_V;

  beacon_data[8] = s2s_beacon_type_b.SOL_P1_C;
  beacon_data[9] = s2s_beacon_type_b.SOL_P2_C;
  beacon_data[10] = s2s_beacon_type_b.SOL_P3_C;
  beacon_data[11] = s2s_beacon_type_b.SOL_P4_C;
  beacon_data[12] = (s2s_beacon_type_b.GYRO_X >> 8) & 0xff;
  beacon_data[13] = (s2s_beacon_type_b.GYRO_X) & 0xff;
  beacon_data[14] = (s2s_beacon_type_b.GYRO_Y >> 8) & 0xff;
  beacon_data[15] = (s2s_beacon_type_b.GYRO_Y) & 0xff;
  beacon_data[16] = s2s_beacon_type_b.GYRO_Z >> 8 & 0xff;
  beacon_data[17] = s2s_beacon_type_b.GYRO_Z & 0xff;

  beacon_data[18] = (s2s_beacon_type_b.ACCL_X >> 8) & 0xff;
  beacon_data[19] = (s2s_beacon_type_b.ACCL_X) & 0xff;
  beacon_data[20] = (s2s_beacon_type_b.ACCL_Y >> 8) & 0xff;
  beacon_data[21] = (s2s_beacon_type_b.ACCL_Y) & 0xff;
  beacon_data[22] = s2s_beacon_type_b.ACCL_Z >> 8 & 0xff;
  beacon_data[23] = s2s_beacon_type_b.ACCL_Z & 0xff;

  beacon_data[24] = (s2s_beacon_type_b.MAG_X >> 8) & 0xff;
  beacon_data[25] = (s2s_beacon_type_b.MAG_X) & 0xff;
  beacon_data[26] = (s2s_beacon_type_b.MAG_Y >> 8) & 0xff;
  beacon_data[27] = (s2s_beacon_type_b.MAG_Y) & 0xff;
  beacon_data[28] = s2s_beacon_type_b.MAG_Z >> 8 & 0xff;
  beacon_data[29] = s2s_beacon_type_b.MAG_Z & 0xff;
  beacon_data[30] = s2s_beacon_type_b.CHK_CRC;
}

int gpio_write(uint32_t pin, uint8_t mode)
{

  gpio_config_s gpio_numval;
  int fd = open(ETX_LED_DRIVER_PATH, O_WRONLY);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Error opening %s for GPIO WRITE...", ETX_LED_DRIVER_PATH);
    close(fd);
    return -1;
  }
  gpio_numval.gpio_num = pin;
  gpio_numval.gpio_val = mode;
  if (gpio_numval.gpio_val > 1 || gpio_numval.gpio_num < 0)
  {
    syslog(LOG_ERR, "Undefined GPIO pin or set mode selected...\n");
    return -2;
  }
  int ret = write(fd, (const void *)&gpio_numval, sizeof(gpio_config_s));
  close(fd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Unable to write to gpio pin...\n");
  }
  return ret;
}
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
  // printf("mq : %d",mq);
  if (mq == (mqd_t)-1)
  {
    // perror("mq_open");
    // exit(1);
  }

  else
  {
    printf("Waiting for messages...\n");

    // while(1)

    // Receive the message
    bytes_read = mq_receive(mq, buffer, MAX_SIZE, NULL);
    if (bytes_read == -1)
    {
      perror("mq_receive");
      // exit(1);
    }

    buffer[bytes_read] = '\0'; // Null-terminate the string

    printf("Received: %s\n", buffer);

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
}

void CHECK_GPIO_1()
{
  reader_mq_2(gpio_name_1);
  uint8_t pin_mode[5] = {0, 0, 0, 0, 0};
  printf("GPIO name is %s\n", gpio_name_1);
  if (!strcmp(gpio_name_1, "COM"))
  {
    gpio_write(GPIO_3V3_COM_EN, !pin_mode[0]);
  }
  else if (!strcmp(gpio_name_1, "MSN1"))
  { // ADCS
    gpio_write(GPIO_MSN_3V3_EN, !pin_mode[3]);
    gpio_write(GPIO_MSN1_EN, !pin_mode[3]);
  }
  else if (!strcmp(gpio_name_1, "MSN2"))
  { // CAM
    gpio_write(GPIO_MSN_3V3_EN, !pin_mode[2]);
    gpio_write(GPIO_MSN2_EN, !pin_mode[2]);
  }
  else if (!strcmp(gpio_name_1, "MSN3"))
  { // EPDM
    gpio_write(GPIO_MSN_3V3_EN, !pin_mode[3]);
    gpio_write(GPIO_MSN3_EN, !pin_mode[3]);
  }
  else if (!strcmp(gpio_name_1, "MUX"))
  {
    gpio_write(GPIO_MUX_EN, !pin_mode[4]);
    gpio_write(GPIO_SFM_MODE, !pin_mode[4]); // TODO: CHECK PULL UP PULL DOWN MODE FOR MSN and OBC access
  }
  else if (!strcmp(gpio_name_1, "ANT"))
  {
    printf("Starting antenna deployment sequence..\n Turning Burner EN line: %d\n Turning Unreg line: %d\n", pin_mode[5], pin_mode[5]);
    gpio_write(GPIO_BURNER_EN, !pin_mode[5]);
    gpio_write(GPIO_UNREG_EN, pin_mode[5]);
  }
  work_queue(HPWORK, &work_gpio12, CHECK_GPIO_1, NULL, SEC2TICK(2));
}

void first()
{

  int ret = work_queue(HPWORK, &work_gpio1, first, NULL, SEC2TICK(2));
  if (ret < 0)
  {
    printf("Failed to queue work\n");
    // return -1;
  }
}

/****************************************************************************
 * Send beacon data to COM
 * To be done every 90 seconds
 ****************************************************************************/
int send_beacon_data()
{
  if (COM_BUSY == 1)
  {
    return -1;
  }
  uint8_t beacon_data[BEACON_DATA_SIZE];
  switch (beacon_type)
  {
  case 0:

    serialize_beacon_a(beacon_data);

    beacon_data[1] = 0xb1;
    beacon_data[2] = 0x51;
    beacon_data[83] = 0x7e;
    break;
  case 1:
    serialize_beacon_b(beacon_data);
    beacon_data[1] = 0xb2;
    beacon_data[2] = 0x51;
    break;
  default:
    printf("wrong case selected\n");
    return -1;
    break;
  }
  beacon_data[0] = 0x53;
  beacon_data[83] = 0x7e;

  beacon_data[84] = '0';
  int fd = open(COM_UART, O_WRONLY);
  if (fd < 0)
  {
    printf("unable to open: %s\n", COM_UART);
    return -1;
  }

  printf("Turning on  4v dcdc line..\n");
  gpio_write(GPIO_DCDC_4V_EN, 1);
  printf("Turning on COM 4V line..\n");
  gpio_write(GPIO_COM_4V_EN, 1);

  int ret = write(fd, beacon_data, BEACON_DATA_SIZE);
  usleep(10000);
  if (ret < 0)
  {
    printf("unable to send data\n");
    for (int i = 0; i < BEACON_DATA_SIZE; i++)
    {
      ret = write(fd, &beacon_data[i], 1);
      usleep(1000);
    }
    if (ret < 0)
    {
      printf("Unable to send data through byte method..\n");
      return -1;
    }
  }
  /*To delete*/
  if (beacon_status == 0)
  {
    printf("\nbeacon 1:\n");
  }
  else
  {
    printf("\nbeacon 2:\n");
  }
  beacon_type = !beacon_type;

  for (int i = 0; i < 83; i++)
  {
    {
      printf("%d ", beacon_data[i]);
    }
  }
  printf("\n");
  /*To delete*/
  int x = 0;
  while (x < 200000)
  {
    x += 80;
    usleep(80);
  }

  // printf("Turning off  4v DCDC line..\n");
  gpio_write(GPIO_DCDC_4V_EN, 0);
  // printf("Turning off COM 4V line..\n");
  gpio_write(GPIO_COM_4V_EN, 0);
  ioctl(fd, TCFLSH, 2);
  ioctl(fd, TCDRN, 2);
  printf("TX RX buffer flused\n");
  close(fd);
  printf("Turned off COM 4V line..\n");
  printf("Beacon Type %d sequence complete\n", beacon_type);
  work_queue(HPWORK, &work_beacon, send_beacon_data, NULL, SEC2TICK(BEACON_DELAY));

  return 0;
}
int receive_telecommand_rx(uint8_t *COM_RX_DATA)
{
  uint8_t useful_command[12];
  uint8_t main_cmd[3] = {'\0'};
  uint16_t rsv_table = 0;
  uint32_t address = 0;
  uint16_t pckt_no = 0;
  uint8_t ack[85] = {0x53, 0xac, 0x04, 0x01, 0x62, 0x63, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x70, 0x71, 0x72, 0x7e, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x80, 0x7e};
  printf("waiting for telecommands from COM\n");
  int ret = receive_data_uart(COM_UART, COM_RX_DATA, COM_RX_CMD_SIZE); // telecommand receive
  if (ret < 0)
  {
    printf("data not received from COM\n NACK sending\n");
    send_data_uart(COM_UART, NACK, 7);
    printf("data not received from COM\n NACK 1s55 ent\n");
    return ret;
  }
  else
  {
    int header;
    // df ab d1 - on_
    // ba do - off
    if (COM_RX_DATA[16] == 0x02)
    {
      header = 16;
    }
    else
    {
      header = 17;
    }
    {
      if (COM_RX_DATA[header + 1] == 0xdf & COM_RX_DATA[header + 2] == 0xab & COM_RX_DATA[header + 2] == 0xd1)
      {
        // gpio_write(GPIO_COM_4V_EN, 1);
        printf("\n ********************Digipeater mode on********************\n");
        digipeating = 1;
        // sleep(3);
        // gpio_write(GPIO_COM_4V_EN, 0);

        return 29;
      }
      if (COM_RX_DATA[header + 1] == 0xdf & COM_RX_DATA[header + 2] == 0xba & COM_RX_DATA[header + 2] == 0xd0)
      {
        printf("\n ********************Digipeater mode off********************\n");
        digipeating = 0;
        return 29;
      }
    }
    // for (int i = 0; i < BEACON_DATA_SIZE; i++)
    // {
    //     send_data_uart(COM_UART, ack[i], 1);
    //     printf("%02x ",ack[i]);
    // }
  }
  printf("value of ret is %d\ndata received from COM\n sending ACK\n", ret);
  sleep(3);

  // ret = send_data_uart(COM_UART, ack, BEACON_DATA_SIZE);

  // ret = send_data_uart(COM_UART, ACK, 7); // ack send

  // print_rx_telecommand(COM_RX_DATA); // printing the received telecommand
  for (int i = 0; i < 12; i++)
  {
    useful_command[i] = COM_RX_DATA[i + 8]; // extracting useful commands
  }
  if (useful_command[6] != 0xff || useful_command[6] != 0x00 | useful_command[7] != 0xff || useful_command[7] != 0x00)
  {
    int x;
    printf("Reservation command received\n"); // if reservation command is received then store the reservation command (do not execute)
    // send_data_uart(COM_UART, ack, BEACON_DATA_SIZE);
    int fd = open(COM_UART, O_WRONLY);
    if (fd < 0)
    {
      printf("unable to open: %s\n", COM_UART);
      return -1;
    }
    printf("Turning on 4v  dcdc  line..\n");
    gpio_write(GPIO_DCDC_4V_EN, 1);
    printf("Turning on 4v RF line..\n");

    gpio_write(GPIO_COM_4V_EN, 1);

    int ret = write(fd, ack, BEACON_DATA_SIZE);
    for (int i = 0; i < 85; i++)
    {
      printf("%02x ", ack[i]);
    }
    close(fd);
    x = 0;
    while (x < 300000)
    {
      x += 80;
      usleep(80);
    }
    printf("Turning off  4v RF switch line..\n");
    gpio_write(GPIO_COM_4V_EN, 0);

    printf("Turning off  4v DCDC line..\n");
    gpio_write(GPIO_DCDC_4V_EN, 0);

    printf("Turning on  4v DCDC line..\n");
    printf("\nACK sent success\n******Sleeping*******\n");
    ack[0] = 0x53;
    ack[1] = 0x0e;
    ack[2] = 0x51;
    for (int i = 3; i < 83; i++)
    {
      ack[i] = i;
    }
    ack[83] = 0x7e;
    int j;
    for (j = 0; j < 10; j++)
    {
      printf("\n EPDM data packet no %d\n", j + 1);

      fd = open(COM_UART, O_WRONLY);
      if (fd < 0)
      {
        printf("unable to open: %s\n", COM_UART);
        return -1;
      }
      printf("Turning on 4v  dcdc  line..\n");
      gpio_write(GPIO_DCDC_4V_EN, 1);
      printf("Turning on 4v RF line..\n");

      gpio_write(GPIO_COM_4V_EN, 1);
      ret = write(fd, ack, BEACON_DATA_SIZE);
      x = 0;
      while (x < 500000)
      {
        x += 100;
        usleep(100);
      }
      printf("Turning of 4v RF line..\n");

      gpio_write(GPIO_COM_4V_EN, 0);
      printf("Turning off 4v dcdc EN line..\n");

      gpio_write(GPIO_DCDC_4V_EN, 0);

      for (int i = 0; i < 85; i++)
      {
        printf("%02x ", ack[i]);
      }
      close(fd);
      x = 0;
      while (x < 100000)
      {
        x += 100;
        usleep(100);
      }
      // sleep(3);

      // send_data_uart(COM_UART, ack, BEACON_DATA_SIZE);
      // send_beacon_data();
      printf("\n EPDM data sent success\n ******Sleeping *******\n ");
    }
    sleep(3);
  }
  else
  {
    switch (useful_command[0])
    {
    case 1:
      printf("command received for OBC\n");
      OBC_CMD_EXE(useful_command);
      break;
    case 2:
      printf("command received for CAM\n");
      break;
    default:
      printf("unknown command\n");
      break;
    }
  }
  return ret;
}

/****************************************************************************
 * COM TASK task
 *
 * COM will be in digipeater mode till a digipeating message is received and digipeated
 ****************************************************************************/
void digipeater_mode(uint8_t *data)
{
  receive_data_uart(COM_UART, data, 29);
  for (int i = 29; i < 84; i++)
  {
    data[i] = 0xff;
  }
  // send_data_uart(COM_UART, data, 84);
  /*To delete*/
  if (send_data_uart(COM_UART, data, 84) > 0)
  {
    printf("digipeating data is \n ");
    for (int i = 0; i < 85; i++)
      printf("%02x ", data);
    printf("digipeating successful\n :");
  }
  /*To delete*/
}
static int COM_TASK(int argc, char *argv[])
{
  int ret = -1;
  uint8_t rx_data[COM_RX_CMD_SIZE] = {'\0'};
  printf("Turning on COM MSN...\n");
  gpio_write(GPIO_3V3_COM_EN, 1);
  usleep(2000000);
  ret = handshake_COM(data); // tx rx data is flushed before closing the file
  usleep(PRINT_DELAY * 100);
  if (ret == 0)
  {
    printf("Successful handshake with COM\n");
  }
  if (ret != 0)
  {
    printf("Unable to handshake with COM\n");
  }
  usleep(1000);

  send_beacon_data();
  printf("Beacon 1 sent...\n");
  printf("Going to receiver mode...\n");
  receive_telecommand_rx(rx_data);
  sleep(10);
  send_beacon_data();
  printf("Beacon 2 sent...\n");
  receive_telecommand_rx(rx_data);
  sleep(2);
  printf("Startign digipeating mode:\n");
  // digipeater_mode(rx_data);
  for (;;)
  {
    receive_telecommand_rx(rx_data);
    usleep(1000);
  }
}

/****************************************************************************
 * COM handshake function
 ****************************************************************************/
int handshake_COM(uint8_t *ack)
{
  double fd;
  uint8_t data1[ACK_DATA_SIZE] = {'\0'};
  int i;
  int count = 0, ret;
  printf("Opening uart dev path : %s ret : %d", COM_UART, fd);
  usleep(PRINT_DELAY);
  fd = open(COM_UART, O_RDWR);
  if (fd < 0)
  {
    printf("error opening %s\n", COM_UART);
    usleep(PRINT_DELAY);
    return -1;
  }

  int wr1 = write(fd, data, ACK_DATA_SIZE); // writing handshake data
  if (wr1 < 0)
  {
    printf("Unable to send data through %d UART", COM_UART);
    usleep(PRINT_DELAY);
    return -1;
  }
  printf("\n%d bytes written\n", wr1);
  usleep(PRINT_DELAY);
  // ret = read(fd, data1, 10);   //try reading data from UART at once as well
  for (i = 0; i < ACK_DATA_SIZE; i++)
  {
    ret = read(fd, &data1[i], 1);
  }
  printf("data received from %s \n", COM_UART);
  usleep(PRINT_DELAY);
  for (int i = 0; i < ACK_DATA_SIZE; i++)
  {
    printf(" %x ", data1[i]);
  }
  printf("\n");
  usleep(PRINT_DELAY);
  if (data[0] == data1[0] && data[ACK_DATA_SIZE - 2] == data1[ACK_DATA_SIZE - 2])
  {
    printf("\n******Acknowledgement received******\n");
    usleep(PRINT_DELAY);
  }
  printf("handshake complete\n");
  printf("\n");
  ioctl(fd, TCFLSH, 2);
  ioctl(fd, TCDRN, NULL);
  printf("flused tx rx buffer\n");
  close(fd);
  return 0;
}

/****************************************************************************
 * MSN handshake function
 ****************************************************************************/
int handshake_MSN(char *uart_msn, uint8_t *ack)
{
  double fd;
  uint8_t data1[ACK_DATA_SIZE] = {'\0'};
  int i;
  int count = 0, ret;
  printf("Opening uart dev path : %s ret : %d", uart_msn, fd);
  usleep(PRINT_DELAY);
  fd = open(uart_msn, O_RDWR);
  if (fd < 0)
  {
    printf("error opening %s\n", uart_msn);
    usleep(PRINT_DELAY);
    return -1;
  }

  int wr1 = write(fd, data, ACK_DATA_SIZE); // writing handshake data
  if (wr1 < 0)
  {
    printf("Unable to send data through %d UART", uart_msn);
    usleep(PRINT_DELAY);
    return -1;
  }
  printf("\n%d bytes written\n", wr1);
  usleep(PRINT_DELAY);
  // ret = read(fd, data1, 10);   //try reading data from UART at once as well

  for (i = 0; i < ACK_DATA_SIZE; i++)
  {
    ret = read(fd, &data1[i], 1);
  }
  printf("data received from %s \n", uart_msn);
  usleep(PRINT_DELAY);
  for (int i = 0; i < ACK_DATA_SIZE; i++)
  {
    printf(" %x ", data1[i]);
  }
  printf("\n");
  usleep(PRINT_DELAY);
  if (data[0] == data1[0] && data[ACK_DATA_SIZE - 2] == data1[ACK_DATA_SIZE - 2])
  {
    printf("\n******Acknowledgement received******\n");
    usleep(PRINT_DELAY);
  }
  printf("handshake complete\n");
  printf("\n");
  ioctl(fd, TCFLSH, 2);
  ioctl(fd, TCDRN, NULL);
  printf("flused tx rx buffer\n");
  close(fd);
  return 0;
}

int main(int argc, FAR char *argv[])
{
  // first();
  // CHECK_GPIO_1();
  if (argc <= 2)
  {
    uint8_t data_retrieved[100];
    retrieve_data_from_flash_edited("/mnt/fs/mfm/mtd_mainstorage", "/satHealth.txt", &data_retrieved, argv[2]);
    for (int i = 0; i < 100; i++)
    {
      printf("%d : %02x      ", i, data_retrieved[i]);
    }
  }
  if (argc < 3)
  { 
    printf("Enter both subsystem name and gpio mode\n Enter: Application Name, Subsystem name (CAM, MSN1, MSN2, MSN3 etc.),GPIO Pin Mode (1 or 0)\n ");
    printf("In case of MUX: 1 for OBC controls FLASH \t 0 to let MSN access FLASH\n");
    return -1;
  }
  // else
  {
    printf("some else called");

    uint8_t pin_mode = atoi(argv[2]);
    printf("Pin mode is %d", pin_mode);
    if (!strcmp(argv[1], "COM"))
    {
      // gpio_write(GPIO_3V3_COM_EN, pin_mode);
      // gpio_write(GPIO_DCDC_4V_EN, pin_mode);
      // COM_TASK(argc,argv);
      if (pin_mode == 0)
      {
        gpio_write(GPIO_3V3_COM_EN, 0);
        gpio_write(GPIO_DCDC_4V_EN, 0);
        // return 0;
      }
      else
      {
      }
      int retval = task_create("task1", 100, 1024, COM_TASK, NULL);
      if (retval < 0)
      {
        printf("unable to create COM task\n");
        return -1;
      }
    }
    else if (!strcmp(argv[1], "MSN1"))
    { // ADCS
      gpio_write(GPIO_MSN_3V3_EN, pin_mode);
      gpio_write(GPIO_MSN1_EN, pin_mode);
      // save_data_to_flash();
      uint8_t data1[] = {'T', 'E', 'S', 'T'};
      save_data_to_flash("/mnt/fs/mfm/mtd_mission", "/cam.txt", &data1);
    }
    else if (!strcmp(argv[1], "MSN2") | !strcmp(argv[1], "CAM"))
    {
      uint8_t rec1;
      uint8_t rec[1000] = {0}; // Initialize to 0 instead of 'NULL'
      uint8_t cam[7] = {0x53, 'O', 'B', 'C', 0x01, 0x7e};
      int fd, ret1, i1 = 0;
      int j1 = 1;

      fd = open(EPDM_UART, O_RDWR);
      if (fd < 0)
      {
        perror("Failed to open UART");
        return -1;
      }

      ret1 = write(fd, cam, sizeof(cam));
      if (ret1 < 0)
      {
        perror("Failed to write to UART");
        close(fd);
        return -1;
      }
      else
      {
        printf("cmd %s sent successfully\n", cam);
      }

      while (rec[i1] != 0xd9 && rec[i1 - 1] != 0xff)
      {
        ret1 = read(fd, &rec1, 1);
        if (ret1 < 0)
        {
          perror("Failed to read from UART");
          close(fd);
          return -1;
        }
        else
        {
          // printf("received data %d\n", i1);
        }
        rec[i1++] = rec1;

        if (i1 % 1001 > 999 || i1 > 1000)
        {
          printf("%d  ", i1);
        }

        if (i1 % 1001 > 999 || i1 > 1000)
        {

          i1 = 0;
          j1++;
          cam[4] = j1;
          for (int p = 0; p < 10; p++)
          {
            printf("%02x ", p);
          }
          save_data_to_flash("/mnt/fs/mfm/mtd_mission", "/cam.txt", &rec);

          // save_data_to_flash("/mnt/fs/mfm/mtd_mission", "/cam.txt", rec);
          sleep(1);
          memset(rec, 0, sizeof(rec)); // Clear the buffer
          ret1 = write(fd, cam, sizeof(cam));
          if (ret1 < 0)
          {
            perror("Failed to write to UART");
            close(fd);
            return -1;
          }

          else
            printf("J is %d \nCommand sent is %s\n", j1, cam);
        }
      }

      printf("\nEnded data reading\n");
      close(fd);
      // fd=open(EPDM_UART, O_RDONLY);
      // printf("Started data reading\n");
      // while(1){
      //   ret1 = read(fd, rec1, sizeof(rec1));
      //   if(ret1 > 0){
      //     printf("%02x ",rec1);
      //   }
      // }

      // printf("\nENDed data reading\n");
      // close(fd);
      // // CAM
      // // gpio_write(GPIO_MSN_3V3_EN, pin_mode);
      // printf("Turning on 3v3 mission pin");
      // // gpio_write(GPIO_MSN2_EN, pin_mode);
      // printf("Turning on MSN2 ...CAM  enable pin\n");
      // printf("Handshaking with CAM  starting \n");
      // uint8_t command[] = {0x53, 'M', 'O', 'D', 'E', 0x7e};
      // int data[200];
      // int i = 0, ret;
      // do
      // {
      //   // send_data_uart(EPDM_UART, command, sizeof(command));
      //   fd = open(EPDM_UART, O_WRONLY);
      //   int ret = write(fd, command, sizeof(command));
      //   if (ret > 5)
      //   {
      //     printf("data of size %d has been sent\n");
      //   }
      //   close(fd);
      //   usleep(50000);
      //   fd = open(EPDM_UART, O_RDONLY);
      //   if (fd < 0)
      //   {
      //     printf("%s opening failed in %d attempt.\n", EPDM_UART, i + 1);
      //   }
      //   else
      //   {

      //     // ret = read(fd, rec, sizeof(rec));
      //     for (int i = 0; i < 6; i++)
      //     {
      //       ret = read(fd, &rec[i], 1);
      //     }
      //     printf("waiting data in uart %d\n", EPDM_UART);
      //     if (ret > 0)
      //     {
      //       printf("Data received %s\n", rec);
      //       close(fd);
      //       break;
      //     }
      //   }
      //   sleep(1);
      // } while (1);
      // sleep(4);
      // if (send_data_uart(EPDM_UART, command, sizeof(command)))
      // {
      //   printf("%s command has been sent successfully\n", command);
      //   usleep(10000);
      //   fd = open(EPDM_UART, O_RDONLY);
      //   if (fd < 0)
      //   {
      //     printf("%s opening failed in %d attempt.\n", EPDM_UART, i + 1);
      //   }
      //   else
      //   {
      //     int timeout = 0;
      //     fd = open(EPDM_UART, O_WRONLY);
      //     do
      //     {
      //       timeout++;
      //       ret = read(fd, data, sizeof(data));
      //       sleep(1);
      //     } while (ret == -1);
      //     while (ret == -1)
      //     {
      //       timeout++;
      //       ret = read(fd, data, sizeof(data));
      //       sleep(1);
      //     }
      //     if (ret > 000)
      //     {
      //       printf("Data of size %d has been received\n", ret);

      //       close(fd);
      //       for (fd = 0; fd < ret; fd++)
      //       {
      //         printf("%c", data[fd]);
      //       }
      //       printf("\n");
      //     }
      //     close(fd);
      //   }
      // }
    }
    else if (!strcmp(argv[1], "MSN3"))
    { // EPDM
      gpio_write(GPIO_MSN_3V3_EN, pin_mode);
      gpio_write(GPIO_MSN3_EN, pin_mode);
    }
    else if (!strcmp(argv[1], "MUX"))
    {
      gpio_write(GPIO_MUX_EN, 1);
      gpio_write(GPIO_SFM_MODE, pin_mode); // TODO: CHECK PULL UP PULL DOWN MODE FOR MSN and OBC access
    }
    else if (!strcmp(argv[1], "ANT"))
    {
      printf("Starting antenna deployment sequence..\n Turning Burner EN line: %d\n Turning Unreg line: %d\n", pin_mode, pin_mode);
      gpio_write(GPIO_BURNER_EN, pin_mode);
      gpio_write(GPIO_UNREG_EN, pin_mode);
    }
    else
    { // keep on adding other gpio pins as you go
      printf("Unknown command \n");
      return -2;
    }
  }
  return 0;
}