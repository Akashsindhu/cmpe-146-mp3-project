#include "uart_lab.h"
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "gpio.h"
#include "lpc40xx.h"
#include "lpc_peripherals.h"
#include "queue.h"

// Private queue handle of our uart_lab.c
static QueueHandle_t your_uart_rx_queue;

void uart_lab__init(uart_number_e uart, uint32_t peripheral_clock, uint32_t baud_rate) {
  // Refer to LPC User manual and setup the register bits correctly
  // a) Power on Peripheral
  // b) Setup DLL, DLM, FDR, LCR registers
  if (uart == 2) {
    LPC_SC->PCONP |= (1 << 24);
  } else if (uart == 3) {
    LPC_SC->PCONP |= (1 << 25);
  }
  const float offset = 0.5;
  const uint16_t div = (uint16_t)((peripheral_clock / (16 * baud_rate)) + offset);
  const uint8_t dlab_bit = (1 << 7);
  const uint8_t eight_bits = (3 << 0);
  const uint8_t two_stop_bits = (1 << 2);

  if (uart == 2) {
    LPC_UART2->LCR |= dlab_bit;

    LPC_UART2->DLM = (div >> 8) & 0xFF;
    LPC_UART2->DLL = (div >> 0) & 0xFF;

    LPC_UART2->LCR &= ~(dlab_bit);
    LPC_UART2->FDR = (1 << 4);
    LPC_UART2->LCR = eight_bits | two_stop_bits;
  }

  else if (uart == 3) {
    LPC_UART3->LCR |= dlab_bit;

    LPC_UART3->DLM = (div >> 8) & 0xFF;
    LPC_UART3->DLL = (div >> 0) & 0xFF;

    LPC_UART3->LCR &= ~(dlab_bit);
    LPC_UART3->FDR = (1 << 4);
    LPC_UART3->LCR |= eight_bits | two_stop_bits;
  }
}

bool uart_lab__polled_get(uart_number_e uart, char *input_byte) {
  // a) Check LSR for Receive Data Ready
  // b) Copy data from RBR register to input_byte
  bool status = false;
  uint8_t check_data_ready = (1 << 0);

  if (uart == 2) {
    while (!(LPC_UART2->LSR & check_data_ready)) {
    }
    *input_byte = LPC_UART2->RBR;
    status = true;
  }

  else if (uart == 3) {
    while (!(LPC_UART3->LSR & check_data_ready)) {
    }
    *input_byte = LPC_UART3->RBR;
    status = true;
  }

  return status;
}

bool uart_lab__polled_put(uart_number_e uart, char output_byte) {
  // a) Check LSR for Transmit Hold Register Empty
  // b) Copy output_byte to THR register
  bool status = false;
  uint8_t transmit_hold_empty = (1 << 5);

  if (uart == 2) {
    while (!(LPC_UART2->LSR & transmit_hold_empty)) {
    }
    LPC_UART2->THR = output_byte;
    while (!(LPC_UART2->LSR & transmit_hold_empty)) {
    }
    status = true;
  }

  else if (uart == 3) {
    while (!(LPC_UART3->LSR & transmit_hold_empty)) {
    }
    LPC_UART3->THR = output_byte;
    while (!(LPC_UART3->LSR & transmit_hold_empty)) {
    }
    status = true;
  }

  return status;
}

// Private function of our uart_lab.c
static void your_receive_interrupt2(void) {
  // TODO: Read the IIR register to figure out why you got interrupted
  // TODO: Based on IIR status, read the LSR register to confirm if there is data to be read
  // TODO: Based on LSR status, read the RBR register and input the data to the RX Queue
  if (LPC_UART2->IIR & (2 << 1)) {
    if (LPC_UART2->LSR & (1 << 0)) {
      const char byte = LPC_UART2->RBR;
      xQueueSendFromISR(your_uart_rx_queue, &byte, NULL);
    }
  }
}

static void your_receive_interrupt3(void) {
  // TODO: Read the IIR register to figure out why you got interrupted
  // TODO: Based on IIR status, read the LSR register to confirm if there is data to be read
  // TODO: Based on LSR status, read the RBR register and input the data to the RX Queue
  if (LPC_UART3->IIR & (2 << 1)) {
    if (LPC_UART3->LSR & (1 << 0)) {
      const char byte = LPC_UART3->RBR;
      xQueueSendFromISR(your_uart_rx_queue, &byte, NULL);
    }
  }
}

// Public function to enable UART interrupt
// TODO Declare this at the header file
void uart__enable_receive_interrupt(uart_number_e uart_number) {
  // TODO: Use lpc_peripherals.h to attach your interrupt
  // TODO: Enable UART receive interrupt by reading the LPC User manual
  // Hint: Read about the IER register

  if (uart_number == UART__2) {
    lpc_peripheral__enable_interrupt(LPC_PERIPHERAL__UART2, your_receive_interrupt2);

    LPC_UART2->IER |= (1 << 0);
  }

  else if (uart_number == UART__3) {
    lpc_peripheral__enable_interrupt(LPC_PERIPHERAL__UART3, your_receive_interrupt3);

    LPC_UART3->IER |= (1 << 0);
  }

  // TODO: Create your RX queue
  your_uart_rx_queue = xQueueCreate(10, sizeof(char));
}

// Public function to get a char from the queue (this function should work without modification)
// TODO: Declare this at the header file
bool uart_lab__get_char_from_queue(char *input_byte, uint32_t timeout) {
  return xQueueReceive(your_uart_rx_queue, input_byte, timeout);
}