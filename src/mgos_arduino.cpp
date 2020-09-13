/*
 * Copyright (c) 2014-2017 Cesanta Software Limited
 * All rights reserved
 */

#include "mgos_arduino.h"

#include "mgos_app.h"

#include <Arduino.h>

#include "common/cs_dbg.h"
#include "mongoose.h"

#include "mgos_gpio.h"
#include "mgos_hal.h"
#include "mgos_init.h"
#include "mgos_time.h"
#include "mgos_timers.h"

#ifndef IRAM
#define IRAM
#endif

static inline uint64_t uptime() {
  return (uint64_t)(1000000 * mgos_uptime());
}

unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout) {
  uint64_t startMicros = uptime();

  // wait for any previous pulse to end
  while (state == mgos_gpio_read(pin)) {
    if ((uptime() - startMicros) > timeout) {
      return 0;
    }
  }

  // wait for the pulse to start
  while (state != mgos_gpio_read(pin)) {
    if ((uptime() - startMicros) > timeout) {
      return 0;
    }
  }

  uint64_t start = uptime();

  // wait for the pulse to stop
  while (state == mgos_gpio_read(pin)) {
    if ((uptime() - startMicros) > timeout) {
      return 0;
    }
  }
  return (uint32_t)(uptime() - start);
}

IRAM void pinMode(uint8_t pin, uint8_t mode) {
  switch (mode) {
    case INPUT:
    case INPUT_PULLUP:
      mgos_gpio_set_mode(pin, MGOS_GPIO_MODE_INPUT);
      mgos_gpio_set_pull(pin, (mode == INPUT_PULLUP ? MGOS_GPIO_PULL_UP
                                                    : MGOS_GPIO_PULL_NONE));
      break;
    case OUTPUT:
      mgos_gpio_set_mode(pin, MGOS_GPIO_MODE_OUTPUT);
      break;
    default:
      LOG(LL_ERROR, ("Unsupported mode: %u", mode));
  }
}

IRAM int digitalRead(uint8_t pin) {
  return mgos_gpio_read(pin);
}

IRAM void digitalWrite(uint8_t pin, uint8_t val) {
  mgos_gpio_write(pin, val);
}

uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
    uint8_t value = 0;
    uint8_t i;

    for(i = 0; i < 8; ++i) {
        digitalWrite(clockPin, HIGH);
        if(bitOrder == LSBFIRST)
            value |= digitalRead(dataPin) << i;
        else
            value |= digitalRead(dataPin) << (7 - i);
        digitalWrite(clockPin, LOW);
    }
    return value;
}

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val) {
    uint8_t i;

    for(i = 0; i < 8; i++) {
        if(bitOrder == LSBFIRST)
            digitalWrite(dataPin, !!(val & (1 << i)));
        else
            digitalWrite(dataPin, !!(val & (1 << (7 - i))));

        digitalWrite(clockPin, HIGH);
        digitalWrite(clockPin, LOW);
    }
}

void delay(unsigned int ms) {
  mgos_usleep(ms * 1000);
}

void delayMicroseconds(unsigned int us) {
  mgos_usleep(us);
}

unsigned long millis(void) {
  return mgos_uptime() * 1000;
}

unsigned long micros(void) {
  return mgos_uptime() * 1000000;
}

void interrupts(void) {
  mgos_ints_enable();
}

void noInterrupts(void) {
  mgos_ints_disable();
}

extern "C" {
static mgos_timer_id s_loop_timer;
}

void setup(void) __attribute__((weak));
void setup(void) {
}

void loop(void) __attribute__((weak));
void loop(void) {
  // It's a dummy loop, no need to invoke it.
  mgos_clear_timer(s_loop_timer);
}

extern "C" {
void loop_cb(void *arg) {
  loop();
  (void) arg;
}

bool mgos_arduino_compat_init(void) {
  setup();
  s_loop_timer = mgos_set_timer(0, true /* repeat */, loop_cb, NULL);
  return true;
}

}  // extern "C"
