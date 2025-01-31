/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include <stdio.h>
#include "pico/stdio/driver.h"
#include "pico/stdio.h"
#include "pico/stdio_uart.h"
#include <string.h>

#include <stdlib.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "hardware/uart.h"
#include "circular.h"

#define UART_ID uart1
#define BAUD_RATE 31250 //Midi 31250 ordi 115200
//#define BAUD_RATE 115200 //Midi 31250 ordi 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

extern bool enqueue(CircularBuffer *cb, uint8_t item);
extern bool dequeue(CircularBuffer *cb, uint8_t *item);
extern void initBuffer(CircularBuffer *cb);

/* This MIDI example send sequence of note (on/off) repeatedly. To test on PC, you need to install
 * synth software and midi connection management software. On
 * - Linux (Ubuntu): install qsynth, qjackctl. Then connect TinyUSB output port to FLUID Synth input port
 * - Windows: install MIDI-OX
 * - MacOS: SimpleSynth
 */

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

static int chars_rxed = 0;
static CircularBuffer cb;
//static int char_rxed = 0;

// RX interrupt handler
void on_uart_rx() {
  while (uart_is_readable(UART_ID)) {
    uint8_t ch = uart_getc(UART_ID);
    if (enqueue(&cb, ch)) {
      chars_rxed++;
    } else {
      printf("Buffer is full, cannot enqueue %d\n", ch);
    }
  }
}

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void midi_task(void);
void midi_serial_task(void);
int pico_led_init(void);
void pico_set_led(bool);


/*------------- MAIN -------------*/
int main(void)
{
  bi_decl(bi_program_description("Provide a USB device interface for Serial Port MIDI."));
  bi_decl(bi_2pins_with_names(UART_TX_PIN, "MIDI UART TX", UART_RX_PIN, "MIDI UART RX"));

  initBuffer(&cb);

  board_init();
  printf("Pico MIDI Device to MIDI UART Adapter\r\n");

  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

// Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

// Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    //int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    int UART_IRQ = UART1_IRQ;
    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    // OK, all set up.
    // Lets send a basic string out, and then run a loop and wait for RX interrupts
    // The handler will count them, but also reflect the incoming data back with a slight change!
    uart_puts(UART_ID, "\nHello, uart interrupts\n");

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

  while (1)
  {
    tud_task(); // tinyusb device task
    led_blinking_task();
    midi_serial_task();

  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

void midi_serial_task(void)
{
  //
  // Send received  character to serialport and to USB port
  static uint8_t item;

  uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
  uint8_t const channel   = 0; // 0 for channel 1

  // The MIDI interface always creates input and output port/jack descriptors
  // regardless of these being used or not. Therefore incoming traffic should be read
  // (possibly just discarded) to avoid the sender blocking in IO
  uint8_t packet[4];
  while ( tud_midi_available() ) tud_midi_packet_read(packet);

  // send received note
  while (dequeue(&cb, &item)) {
    uint8_t note_on[3] = { item, 0, 127 };
    //uint8_t note_on[3] = { 0x90 | channel, char_rxed, 127 };
    tud_midi_stream_write(cable_num, note_on, 1);
    //sprintf(packet, "%d", char_rxed);
    //itoa(actual, int_string, 10);
    //printf("%s\n", packet);
    printf("%d\n", item);
    //stdio_uart.out_chars(note_on,1);
    // printf("N");
    chars_rxed--;
    }


}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  // board_led_write(led_state);
  pico_set_led(led_state);
  led_state = 1 - led_state; // toggle
}

int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#endif
}

// Turn the led on or off
void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}