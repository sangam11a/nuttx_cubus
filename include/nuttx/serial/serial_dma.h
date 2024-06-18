#ifndef __INCLUDE_NUTTX_SERIAL_SERIAL_DMA_H
#define __INCLUDE_NUTTX_SERIAL_SERIAL_DMA_H
// #pragma
#include <nuttx/config.h>

#include <assert.h>
#include <sys/types.h>
#include <stdint.h>
#include <debug.h>
#include <nuttx/signal.h>

#include <nuttx/serial/serial.h>

static int uart_recvchars_check_special(FAR uart_dev_t *dev);
void uart_xmitchars_dma(FAR uart_dev_t *dev);
void uart_xmitchars_done(FAR uart_dev_t *dev);

void uart_recvchars_dma(FAR uart_dev_t *dev);
void uart_recvchars_done(FAR uart_dev_t *dev);

#endif