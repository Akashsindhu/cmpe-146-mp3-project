#include <stdbool.h>
#include <stddef.h>

#include "my_ssp0.h"

#include "clock.h"
#include "lpc40xx.h"
#include "lpc_peripherals.h"

void my_ssp0__init(uint32_t max_clock_mhz) {
  // Power on Peripheral
  lpc_peripheral__turn_on_power_to(LPC_PERIPHERAL__SSP0);

  // Setup control registers CR0 and CR1
  LPC_SSP0->CR0 = 7; // Setup for 8-bit transfer while zero-ing all other bits
  LPC_SSP0->CR1 = 2; // Setup for enabling SSP (SPI) while zero-ing all other bits

  // Setup prescalar register to be <= max_clock_mhz
  uint8_t divider = 2; // 8 bit integer to set prescalar register
  const uint32_t cpu_clock_mhz = clock__get_core_clock_hz() / 1000000UL;

  while ((max_clock_mhz < (cpu_clock_mhz / divider)) && divider <= 254) {
    divider += 2;
  }

  LPC_SSP0->CPSR = divider;
}

uint8_t my_ssp0__exchange_byte(uint8_t data_out) {
  // Configure the Data register(DR) to send and receive data by checking the status register
  LPC_SSP0->DR = data_out;

  while (LPC_SSP0->SR & (1 << 4)) {
    // Waiting for Status Register to indicate sending/recieving data is complete
  }

  uint8_t data_return = LPC_SSP0->DR & 0xFF;
  return (data_return);
}
