/*H***********************************************************************
* FILENAME :        uart.c
*
* DESCRIPTION :
* Port of the UART Module to be used in Zephyr on the MAX78000.
*
* LICENSE AND COPYRIGHT:
* SPDX-FileCopyrightText: (c) 2025 Siemens AG
* SPDX-License-Identifier: BSD-3-Clause
* 
* Parts are copied from:
* - sm_clib - Dust Networks - https://github.com/dustcloud/sm_clib
*
* NOTES :
* This application is part of the OpenSwarm Project.
*
* AUTHOR :
* Fabian Graf - fabian.graf@siemens.com
*
* CHANGES :
* VERSION DATE    WHO     DETAIL
* 0       14May25 FG      Initial Commit
*
*H***********************************************************************/

#include "../../dn_uart.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>


//=========================== defines =========================================

#define RING_BUF_SIZE 256
#define UART_THREAD_STACK_SIZE 1024
#define UART_THREAD_PRIORITY 5

K_THREAD_DEFINE(uart_rx_tid, UART_THREAD_STACK_SIZE,
   dn_uart_rx_thread, NULL, NULL, NULL,
   UART_THREAD_PRIORITY, 0, 0);

//=========================== variables =======================================

typedef struct {
   dn_uart_rxByte_cbt   ipmt_uart_rxByte_cb;
} dn_uart_vars_t;

dn_uart_vars_t dn_uart_vars;
uint8_t rxBuf[32];
int bytes_read;

static const struct device *const uart_dev2 = DEVICE_DT_GET(DT_NODELABEL(uart2));

RING_BUF_DECLARE(uart_rx_ringbuf, RING_BUF_SIZE);


//=========================== prototypes ======================================

//=========================== interrupt handlers ==============================

void isr_handler_uart2(){
   if (!uart_irq_update(uart_dev2)) {
		return;
	}

   if (uart_irq_rx_ready(uart_dev2)) {
      while ((bytes_read = uart_fifo_read(uart_dev2, rxBuf, sizeof(rxBuf))) > 0) {
          int stored = ring_buf_put(&uart_rx_ringbuf, rxBuf, bytes_read);
          if (stored < bytes_read) {
              printk("Ring buffer overflow! Lost %d bytes\n", bytes_read - stored);
          }
      }
  }
}

//=========================== public ==========================================

void dn_uart_init(dn_uart_rxByte_cbt rxByte_cb){
   
   // call back function
   dn_uart_vars.ipmt_uart_rxByte_cb = rxByte_cb;
   // Define ISR for reception
   uart_irq_callback_user_data_set(uart_dev2, isr_handler_uart2, NULL);
   
   if (device_is_ready(uart_dev2)) {
      printk("UART device ready!\n");
   } else {
      printk("UART device not ready!\n");
   }
   // Enable RX Interrupt
   uart_irq_rx_enable(uart_dev2);
}

void dn_uart_txByte(uint8_t byte){

  uart_poll_out(uart_dev2, byte);
  printk("TX: 0x%02X\n", byte);
}

void dn_uart_rx_thread(void)
{
    uint8_t buf[32];
    uint32_t len;

    while (1) {
        len = ring_buf_get(&uart_rx_ringbuf, buf, sizeof(buf));
        if (len > 0) {
            printk("RX:");
            for (int i = 0; i < len; i++) {
                if (i < len - 1)
                    printk(" 0x%02X,", buf[i]);
                else
                    printk(" 0x%02X\n", buf[i]);

                dn_uart_vars.ipmt_uart_rxByte_cb(buf[i]);
            }
        } else {
            k_sleep(K_MSEC(5));
        }
    }
}

void dn_uart_txFlush(){
   // nothing to do since nRF driver is byte-oriented
}

//=========================== private =========================================

//=========================== helpers =========================================

