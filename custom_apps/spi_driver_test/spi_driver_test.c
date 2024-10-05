#include <nuttx/wqueue.h>
#include <nuttx/config.h>
#include <mqueue.h>
#include <stdio.h>
#include <fcntl.h>
#include <syslog.h>

#define QUEUE_NAME_1 "/sat_health"
#define QUEUE_NAME_2 "/flags"
#define QUEUE_NAME_3 "/seek_pointer"
#define MAX_SIZE 1024
#define MSG_STOP "//exit"

static struct work_s work_gpio1;

void reader_mq_2(void *arg)
{
  struct mq_attr attr;
  attr.mq_flags = 0;
  attr.mq_maxmsg = 1000;
  attr.mq_msgsize = 8192;
  attr.mq_curmsgs = 0;

  const char *queue_names[] = {QUEUE_NAME_1, QUEUE_NAME_2, QUEUE_NAME_3};
  char buffer[MAX_SIZE + 1];

  for (int j = 0; j < sizeof(queue_names) / sizeof(queue_names[0]); j++)
  {
    mqd_t mq = mq_open(queue_names[j], O_RDONLY, 0644, &attr);
    if (mq == (mqd_t)-1)
    {
      syslog(LOG_DEBUG, "Failed to open queue: %s\n", queue_names[j]);
      continue;
    }

    ssize_t bytes_read = mq_receive(mq, buffer, MAX_SIZE, NULL);
     syslog(LOG_DEBUG, "Bytes read : %d",bytes_read);
    if (bytes_read == -1)
    {
      syslog(LOG_DEBUG, "Failed to receive message from queue: %s\n", queue_names[j]);
    }
    else
    {
      buffer[bytes_read] = '\0'; // Null-terminate the string
      syslog(LOG_DEBUG, "Received: %s from queue: %s\n", buffer, queue_names[j]);
    }

    if (mq_close(mq) == -1)
    {
      syslog(LOG_DEBUG, "Failed to close queue: %s\n", queue_names[j]);
    }
    else
    {
      syslog(LOG_DEBUG, "Queue %s closed successfully\n", queue_names[j]);
    }

    if (mq_unlink(queue_names[j]) == -1)
    {
      syslog(LOG_DEBUG, "Failed to unlink queue: %s\n", queue_names[j]);
    }
    else
    {
      syslog(LOG_DEBUG, "Queue %s unlinked successfully\n", queue_names[j]);
    }
  }

  if (work_queue(HPWORK, &work_gpio1, reader_mq_2, NULL, SEC2TICK(14)) < 0)
  {
    syslog(LOG_DEBUG, "Failed to queue work\n");
  }
}

int main(int argc, FAR char *argv[])
{
  int ret = task_create("data_reader", 100, 2080, (main_t)reader_mq_2, (FAR char *const *)NULL);
  if (ret < 0)
  {
    syslog(LOG_DEBUG, "Unable to create task data_reader\n");
    return -1;
  }
  syslog(LOG_DEBUG, "SPI daemon running\n");

  return 0;
}
