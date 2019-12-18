// @file gpio_isr.c
#include "gpio_isr.h"

#include "lpc40xx.h"

// Note: You may want another separate array for falling vs. rising edge callbacks
static function_pointer_t gpio0_callbacks[32];

void gpio0__attach_interrupt(uint32_t pin, gpio_interrupt_e interrupt_type, function_pointer_t callback) {
  gpio0_callbacks[pin] = callback;
  if (interrupt_type == GPIO_INTR__RISING_EDGE) {
    LPC_GPIOINT->IO0IntEnR |= (1 << pin);
  } else {
    LPC_GPIOINT->IO0IntEnF |= (1 << pin);
  }
}

void gpio0__interrupt_dispatcher(void) {
  // Check which pin generated the interrupt

  int pin_that_generated_interrupt = 0;
  for (int i = 0; i < 32; i++) {
    if ((LPC_GPIOINT->IO0IntStatF & (1 << i)) || (LPC_GPIOINT->IO0IntStatR & (1 << i))) {
      pin_that_generated_interrupt = i;
    }
  }

  function_pointer_t attached_user_handler = gpio0_callbacks[pin_that_generated_interrupt];

  // Invoke the user registered callback, and then clear the interrupt
  attached_user_handler();
  LPC_GPIOINT->IO0IntClr |= (1 << pin_that_generated_interrupt);
}
