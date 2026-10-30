#ifndef HOST_ESP_LINK_H
#define HOST_ESP_LINK_H

#include <stdint.h>

/* USART2 (PA2 = TX, PA3 = RX) <-> ESP32-S3, 115200 8N1.
   Used as a second, alternative command input: the board executes lines that
   arrive either from the CH343P USB-UART (USART1) or from the ESP32 (USART2).
   The two links are not used at the same time; requests are executed in arrival
   order and the reply is routed back to the link the request came from.
   TX is polling based (small volumes), RX uses the RXNE interrupt + ring buffer
   so the main loop never blocks on the link. */

void esp_link_init(uint32_t baud);

/* Returns 1 when one complete line (terminated by CR or LF) is available.
   `line` should be at least 64 bytes. */
int esp_link_read_line(char *line, uint32_t size);

/* Blocking write helpers, used by the console output router. */
void esp_link_puts(const char *text);
void esp_link_putc(char ch);

void esp_link_flush_input(void);
uint32_t esp_link_dropped_bytes(void);
uint32_t esp_link_rx_bytes(void);
/* Lines dropped because they contained non-printable bytes (floating RX). */
uint32_t esp_link_bad_lines(void);
uint32_t esp_link_bad_bytes(void);
/* Lines ignored because they did not start with the ESP command prefix. */
uint32_t esp_link_ignored_lines(void);
/* Pin-level test: drive PA2 low/high, read PA3. out[0]=idle PA2, out[1]=idle PA3,
   out[2]=PA3 with PA2 driven low, out[3]=PA3 with PA2 driven high. A following
   PA3 means TX and RX are shorted (or the ESP32 ties them). */
void esp_link_pin_test(uint32_t out[4]);
/* Call every main-loop pass. Returns 0 ok, 1 currently muted, 2 just muted. */
int esp_link_guard(uint32_t now_ms);
int esp_link_muted(void);
uint32_t esp_link_muted_drops(void);

/* 1 when the UART has been initialised (TX may be used). */
int esp_link_ready(void);

#endif
