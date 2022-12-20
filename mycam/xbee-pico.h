/**
 * Copyright (c) 2009 Andrew Rapp. All rights reserved.
 *
 * This file is part of XBee-Arduino.
 *
 * XBee-Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * XBee-Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with XBee-Arduino.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef XBee_PICO_h
#define XBee_PICO_h

#include <inttypes.h>
#include <stdlib.h>

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define SERIAL_8N1 0x800001c

#define UART0_ID uart0
#define UART1_ID uart1
#define UART_BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1

#define UART1_TX_PIN 4
#define UART1_RX_PIN 5



unsigned long millis();

class Stream {
 public:
  virtual uint8_t available(void) = 0;
  virtual uint8_t read() = 0;
  virtual void write(uint8_t val) = 0;
};

class HardwareSerial: public Stream
{
  uart_inst_t *_uart;
  uint8_t _tx_pin = UART0_TX_PIN;
  uint8_t _rx_pin = UART0_RX_PIN;
  uint8_t _rx_irq = UART0_IRQ;

  void (* _rx_irq_func)();
public:
  HardwareSerial(){};
  HardwareSerial(int uart_nr);
  uint8_t available(void);
  uint8_t read();
  void write(uint8_t val); // { }
  uart_inst_t * getUart();
};

extern HardwareSerial Serial;

//#define NULL 0

#endif //XBee_PICO_h
