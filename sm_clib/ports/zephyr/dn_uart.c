/*H***********************************************************************
* FILENAME :        uart.c
*
* DESCRIPTION :
* Port of the UART Module to be used in Zephyr.
*
* LICENSE AND COPYRIGHT:
* SPDX-FileCopyrightText: (c) 2023 Siemens AG
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
* 0       26Sep23 FG      Initial Commit
* 1       18Oct23 FG      Use UART1 instead of UART0
*
*H***********************************************************************/

#include "../../dn_uart.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>


//=========================== defines =========================================
// In case we are using UART0:
//#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

//=========================== variables =======================================

typedef struct {
   dn_uart_rxByte_cbt   ipmt_uart_rxByte_cb;
} dn_uart_vars_t;

dn_uart_vars_t dn_uart_vars;
uint8_t rxByte;

// In case we are using UART0:
//static const struct device *const uart_dev0 = DEVICE_DT_GET(UART_DEVICE_NODE);
// In case we are using UART1: (TX P0.09 and RX P0.10)
static const struct device *const uart_dev1 = DEVICE_DT_GET(DT_NODELABEL(arduino_serial));

//=========================== prototypes ======================================

//=========================== interrupt handlers ==============================

void isr_handler_uart1(){
   if (!uart_irq_update(uart_dev1)) {
      return;
   }
   // hand over byte to ipmt module
   while (uart_irq_rx_ready(uart_dev1)) {
      uart_fifo_read(uart_dev1, &rxByte, 1);
      dn_uart_vars.ipmt_uart_rxByte_cb(rxByte);
   }
}

//=========================== public ==========================================

void dn_uart_init(dn_uart_rxByte_cbt rxByte_cb){
   
   // call back function
   dn_uart_vars.ipmt_uart_rxByte_cb = rxByte_cb;
   // Define ISR for reception
   uart_irq_callback_user_data_set(uart_dev1, isr_handler_uart1, NULL);
   uart_irq_rx_enable(uart_dev1);
   if (device_is_ready(uart_dev1)) {
      printk("UART1 device ready!\n");
   }

}

void dn_uart_txByte(uint8_t byte){

  uart_poll_out(uart_dev1, byte);
}

void dn_uart_txFlush(){
   // nothing to do since nRF driver is byte-oriented
}

//=========================== private =========================================

//=========================== helpers =========================================

