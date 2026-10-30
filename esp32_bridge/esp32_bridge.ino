/*
 * esp32_bridge.ino - ESP32-S3 <-> G431 host board command bridge
 *
 * Purpose
 *   The G431 host board accepts commands on two inputs: CH343P USB (USART1)
 *   and this ESP32 (USART2, 115200 8N1, PA2/PA3). This sketch turns the
 *   ESP32 into a transparent bridge:
 *
 *       PC terminal  <-- USB CDC -->  ESP32  <-- UART -->  G431
 *
 *   Every line typed on the PC is forwarded to the G431 with the '@' prefix
 *   the G431 requires (see keil5_host/README.md), and every line the G431
 *   sends back is printed on the PC.
 *
 * Wiring (G431 side is fixed, ESP32 side is configurable below)
 *   ESP32 TX  --> G431 PA3  (USART2_RX)
 *   ESP32 RX  <-- G431 PA2  (USART2_TX)
 *   GND       --- GND
 *   Do NOT connect 5V/3V3 between the boards if both are powered separately.
 *
 * Arduino IDE settings
 *   Board              : ESP32S3 Dev Module (or your exact module)
 *   USB CDC On Boot    : Enabled     <-- so Serial is the native USB port
 *   Upload Mode        : UART0/HWCDC or USB-OTG, whichever your board uses
 *   Flash size / PSRAM : defaults are fine
 *
 * Protocol with the G431
 *   TX to G431 : '@' + <command line> + CRLF   (e.g. "@?", "@m", "@c1 T0.5")
 *   RX from G431: plain text lines with CRLF
 *   Baud       : 115200 8N1 (must match HOST_ESP_BAUD on the G431)
 *
 * Local commands (handled by the ESP32 itself, start with '!')
 *   !help            this text
 *   !stat            link statistics
 *   !ping            send "@?" and report the round-trip time
 *   !probe           try every UART/pin combination to find the G431 link
 *   !raw on|off      print the G431 bytes as hex (debugging)
 *   !reboot          restart the ESP32
 *
 * Any other line is forwarded to the G431. You may type the line with or
 * without a leading '@'; the bridge adds exactly one '@'.
 */

#include <Arduino.h>
#include <strings.h>

/* ------------------------------------------------------------------ */
/* configuration                                                       */
/* ------------------------------------------------------------------ */

#define FW_VERSION "1.0.0"

/* Which UART talks to the G431.
 *   0 = UART0 (the ROM bootloader prints here, so if you saw
 *       "invalid header: 0xffffffff" on the G431, the link is UART0)
 *   1 = UART1
 *   2 = UART2
 * On a classic ESP32 board the on-board USB-serial chip owns UART0, so use 1
 * or 2 there. On the ESP32-S3 with native USB (USB CDC On Boot = Enabled),
 * UART0 is free for the G431.
 * If unsure, leave it at 0 and run !probe - it finds the working link. */
#ifndef G431_UART_NUM
#define G431_UART_NUM 0
#endif

/* Pins of the G431 link. -1 means "use this UART's default pins".
 * Remember: ESP32 TX -> G431 PA3, ESP32 RX <- G431 PA2. */
#ifndef G431_TX_PIN
#define G431_TX_PIN -1
#endif
#ifndef G431_RX_PIN
#define G431_RX_PIN -1
#endif

#define G431_BAUD 115200
#define G431_UART_CONFIG SERIAL_8N1

#define LINE_MAX 96            /* longest accepted command line */
#define PROBE_TIMEOUT_MS 300   /* wait for a reply while probing */
#define HEX_DUMP_MAX 24        /* bytes per !raw line */
#define LINK_RETRY_MS 5000u    /* re-check the link when it looks down */
#define BRIDGE_SET_USB_TIMEOUT 1 /* set to 0 if setTxTimeoutMs() is missing */

#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
#define USB_IS_CDC 1
#else
#define USB_IS_CDC 0
#endif

/* ------------------------------------------------------------------ */
/* state                                                               */
/* ------------------------------------------------------------------ */

static HardwareSerial uart0(0);
static HardwareSerial uart1(1);
static HardwareSerial uart2(2);
static HardwareSerial *g431 = nullptr;   /* the link that answered */
static int g431_num = -1;                /* its UART number, for diagnostics */

static char usb_line[LINE_MAX];
static uint32_t usb_line_len;
static char g431_line[LINE_MAX];
static uint32_t g431_line_len;

static bool raw_mode;                    /* hex dump the G431 bytes */
static uint32_t g431_crlf_skip;          /* swallow the LF after a CR */

static uint32_t stat_usb_cmds;           /* lines forwarded to the G431 */
static uint32_t stat_g431_lines;         /* lines received from the G431 */
static uint32_t stat_usb_bytes;
static uint32_t stat_g431_bytes;
static uint32_t stat_dropped;            /* oversize lines */
static uint32_t stat_crlf;
static uint32_t stat_local;
static uint32_t last_reply_ms;
static uint32_t ping_sent_ms;

/* ------------------------------------------------------------------ */
/* tiny USB console helpers                                            */
/* ------------------------------------------------------------------ */

static void usb_write(const char *s)
{
    if (s == nullptr) return;
    Serial.write((const uint8_t *)s, strlen(s));
}

static void usb_printf(const char *fmt, ...)
{
    char buf[192];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
    Serial.write((const uint8_t *)buf, (size_t)n);
}

static HardwareSerial *uart_by_num(uint8_t num)
{
    switch (num) {
    case 0: return &uart0;
    case 1: return &uart1;
    case 2: return &uart2;
    default: return nullptr;
    }
}

/* ------------------------------------------------------------------ */
/* G431 link                                                           */
/* ------------------------------------------------------------------ */

static void g431_send_command(const char *text)
{
    uint32_t n;

    if (g431 == nullptr || text == nullptr) return;
    if (text[0] == '@') ++text;          /* never double the prefix */
    n = (uint32_t)strlen(text);
    if (n == 0u) return;
    g431->write('@');
    g431->write((const uint8_t *)text, n);
    g431->write('\r');
    g431->write('\n');
    ++stat_usb_cmds;
}

/* Read one line from the G431 and print it. Returns true when a line was
 * completed (used by the probe / ping code, which waits for a reply). */
static bool g431_poll(char *out, uint32_t out_size)
{
    bool got = false;

    if (g431 == nullptr) return false;
    while (g431->available() > 0) {
        int v = g431->read();
        uint8_t b;
        if (v < 0) break;
        b = (uint8_t)v;
        ++stat_g431_bytes;

        if (raw_mode) {
            usb_printf("%02X ", b);
        }
        if (b == '\r' || b == '\n') {
            g431_crlf_skip = (b == '\r') ? 2u : 1u;
            if (g431_line_len > 0u) {
                g431_line[g431_line_len] = 0;
                if (out != nullptr && out_size > 0u) {
                    uint32_t n = g431_line_len;
                    if (n >= out_size) n = out_size - 1u;
                    memcpy(out, g431_line, n);
                    out[n] = 0;
                }
                ++stat_g431_lines;
                last_reply_ms = millis();
                got = true;
                if (!raw_mode) {
                    usb_write(g431_line);
                    usb_write("\r\n");
                } else {
                    usb_write("\r\n");
                }
                g431_line_len = 0u;
            }
            continue;
        }
        if (g431_crlf_skip != 0u) {
            /* After a CR the LF must be swallowed, not treated as data. */
            if (g431_crlf_skip == 2u && b == '\n') {
                g431_crlf_skip = 1u;
                continue;
            }
            g431_crlf_skip = 0u;
        }
        if (g431_line_len + 1u < sizeof(g431_line)) {
            g431_line[g431_line_len++] = (char)b;
        } else {
            g431_line_len = 0u;          /* drop a runaway line */
            ++stat_dropped;
        }
    }
    return got;
}

/* Send a command and wait up to timeout_ms for one reply line. */
static bool g431_ask(const char *cmd, char *reply, uint32_t reply_size, uint32_t timeout_ms)
{
    uint32_t t0;
    uint32_t now;

    if (reply != nullptr && reply_size > 0u) reply[0] = 0;
    g431_send_command(cmd);
    t0 = millis();
    while ((uint32_t)(millis() - t0) < timeout_ms) {
        if (g431_poll(reply, reply_size)) return true;
        delay(1);
    }
    return false;
}

static void g431_begin(HardwareSerial *port, int tx_pin, int rx_pin)
{
    /* Arduino-ESP32 signature: begin(baud, config, rxPin, txPin, invert) */
    port->begin(G431_BAUD, G431_UART_CONFIG, rx_pin, tx_pin);
    port->setTimeout(50);
}

static void g431_link_up(HardwareSerial *port, int num)
{
    g431 = port;
    g431_num = num;
    g431_line_len = 0u;
    g431_crlf_skip = 0u;
}

/* ------------------------------------------------------------------ */
/* probing                                                             */
/* ------------------------------------------------------------------ */

/* Try one UART; returns true when the G431 answers "@?" with something
 * that looks like the host board status line. */
static bool probe_one(uint8_t num, int tx_pin, int rx_pin, char *first_line, uint32_t size)
{
    HardwareSerial *port = uart_by_num(num);
    uint32_t t0;
    bool ok = false;

    if (port == nullptr) return false;
    if (first_line != nullptr && size > 0u) first_line[0] = 0;

    g431_begin(port, tx_pin, rx_pin);
    delay(30);
    while (port->available() > 0) (void)port->read();   /* drop stale bytes */

    port->write('@');
    port->write('?');
    port->write('\r');
    port->write('\n');

    t0 = millis();
    while ((uint32_t)(millis() - t0) < PROBE_TIMEOUT_MS) {
        while (port->available() > 0) {
            int v = port->read();
            if (v < 0) break;
            if (v == '\r' || v == '\n') {
                if (g431_line_len > 0u) {
                    g431_line[g431_line_len] = 0;
                    g431_line_len = 0u;
                    ok = true;
                    if (first_line != nullptr && size > 0u) {
                        strncpy(first_line, g431_line, size - 1u);
                        first_line[size - 1u] = 0;
                    }
                    break;
                }
            } else if (g431_line_len + 1u < sizeof(g431_line)) {
                g431_line[g431_line_len++] = (char)v;
            } else {
                g431_line_len = 0u;
            }
        }
        if (ok) break;
        delay(1);
    }
    g431_line_len = 0u;
    (void)ok;
    return ok;
}

static void probe_all(void)
{
    static const uint8_t nums[3] = {0u, 1u, 2u};
    char line[LINE_MAX];
    bool found = false;

    usb_write("[probe] looking for the G431 (115200 8N1, '@?' -> status line)\r\n");
    for (uint8_t i = 0u; i < 3u; ++i) {
        uint8_t num = nums[i];
        if (probe_one(num, G431_TX_PIN, G431_RX_PIN, line, sizeof(line))) {
            usb_printf("[probe] UART%u default pins : OK  \"%s\"\r\n", num, line);
            if (!found) {
                g431_link_up(uart_by_num(num), (int)num);
                found = true;
            }
        } else {
            usb_printf("[probe] UART%u default pins : no reply\r\n", num);
        }
    }
    if (!found) {
        g431 = nullptr;
        usb_write("[probe] nothing answered. Check:\r\n"
                  "  - ESP32 TX -> G431 PA3, ESP32 RX -> G431 PA2, GND common\r\n"
                  "  - G431 powered and running (its USB/CH343P banner appears)\r\n"
                  "  - if the board owns UART0 for its USB bridge, use UART1/2\r\n");
    }
}

/* ------------------------------------------------------------------ */
/* local commands                                                      */
/* ------------------------------------------------------------------ */

static void print_help(void)
{
    usb_write("\r\n=== ESP32 <-> G431 bridge " FW_VERSION " ===\r\n");
    usb_printf("chip=%s  USB CDC=%d  G431 link=UART? baud=%d\r\n",
               ESP.getChipModel(), USB_IS_CDC, G431_BAUD);
    usb_write("Type a G431 command and press Enter, e.g.  ?   m   b   c1 T0.5\r\n"
              "The bridge prefixes it with '@' automatically.\r\n"
              "Local commands:\r\n"
              "  !help   this text\r\n"
              "  !stat   link statistics\r\n"
              "  !ping   send '@?' and measure the round-trip\r\n"
              "  !probe  try every UART to find the G431\r\n"
              "  !raw on|off  hex-dump the G431 bytes\r\n"
              "  !reboot restart the ESP32\r\n");
}

static void print_stat(void)
{
    usb_printf("\r\n[stat] fw=" FW_VERSION " chip=%s usb_cdc=%d\r\n",
               ESP.getChipModel(), USB_IS_CDC);
    usb_printf("[stat] g431 link=%s (UART%d, TX=%d RX=%d) baud=%d raw=%d\r\n",
               (g431 == nullptr) ? "DOWN" : "UP", g431_num, G431_TX_PIN, G431_RX_PIN,
               G431_BAUD, raw_mode ? 1 : 0);
    usb_printf("[stat] cmds=%lu  replies=%lu  usb_bytes=%lu  g431_bytes=%lu\r\n",
               (unsigned long)stat_usb_cmds, (unsigned long)stat_g431_lines,
               (unsigned long)stat_usb_bytes, (unsigned long)stat_g431_bytes);
    usb_printf("[stat] dropped=%lu  local_cmds=%lu  idle_ms=%lu\r\n",
               (unsigned long)stat_dropped, (unsigned long)stat_local,
               (unsigned long)(last_reply_ms == 0u ? 0u : (millis() - last_reply_ms)));
    if (g431 == nullptr) {
        usb_write("[stat] no link - run !probe\r\n");
    } else if (last_reply_ms == 0u || (uint32_t)(millis() - last_reply_ms) > 5000u) {
        usb_write("[stat] link silent for >5 s - is the G431 running?\r\n");
    }
}

static void handle_local(char *line)
{
    ++stat_local;

    if (!strcasecmp(line, "!help") || !strcmp(line, "!")) {
        print_help();
    } else if (!strcasecmp(line, "!stat")) {
        print_stat();
    } else if (!strcasecmp(line, "!probe")) {
        probe_all();
    } else if (!strcasecmp(line, "!ping")) {
        char reply[LINE_MAX];
        uint32_t t0 = millis();
        bool ok;
        ping_sent_ms = t0;
        ok = g431_ask("?", reply, sizeof(reply), 400u);
        if (ok) usb_printf("[ping] OK  %lu ms  \"%s\"\r\n",
                           (unsigned long)(millis() - t0), reply);
        else    usb_printf("[ping] timeout after %lu ms (link=%s)\r\n",
                           (unsigned long)(millis() - t0),
                           (g431 == nullptr) ? "DOWN" : "UP");
    } else if (!strncasecmp(line, "!raw", 4)) {
        const char *arg = line + 4;
        while (*arg == ' ') ++arg;
        if (!strcasecmp(arg, "on")) {
            raw_mode = true;
            usb_write("[raw] on\r\n");
        } else if (!strcasecmp(arg, "off")) {
            raw_mode = false;
            usb_write("[raw] off\r\n");
        } else {
            usb_printf("[raw] usage: !raw on|off   (now=%d)\r\n", raw_mode ? 1 : 0);
        }
    } else if (!strcasecmp(line, "!reboot")) {
        usb_write("[sys] rebooting\r\n");
        Serial.flush();
        delay(50);
        ESP.restart();
    } else {
        usb_printf("[local] unknown '%s' - try !help\r\n", line);
    }
}

/* ------------------------------------------------------------------ */
/* USB (PC) side line assembly                                         */
/* ------------------------------------------------------------------ */

static void usb_handle_byte(uint8_t b)
{
    if (b == '\r' || b == '\n') {
        if (usb_line_len > 0u) {
            usb_line[usb_line_len] = 0;
            if (usb_line[0] == '!') {
                handle_local(usb_line);
            } else if (g431 == nullptr) {
                usb_printf("[bridge] no G431 link - command '%s' dropped (try !probe)\r\n",
                           usb_line);
            } else {
                g431_send_command(usb_line);
            }
            usb_line_len = 0u;
        }
        return;
    }
    if (b == 8u || b == 127u) {
        if (usb_line_len > 0u) --usb_line_len;
        return;
    }
    ++stat_usb_bytes;
    if (b < 0x20u || b > 0x7Eu) return;          /* ignore control bytes */
    if (usb_line_len + 1u < sizeof(usb_line)) {
        usb_line[usb_line_len++] = (char)b;
    } else {
        usb_line_len = 0u;
        ++stat_dropped;
        usb_write("\r\n[bridge] line too long - dropped\r\n");
    }
}

/* ------------------------------------------------------------------ */
/* setup / loop                                                        */
/* ------------------------------------------------------------------ */

void setup(void)
{
    HardwareSerial *port;

    Serial.begin(115200);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
    /* Do not block forever when the PC is not listening yet. */
    {
        uint32_t t0 = millis();
        while (!Serial && (uint32_t)(millis() - t0) < 1500u) delay(10);
    }
#if BRIDGE_SET_USB_TIMEOUT
    Serial.setTxTimeoutMs(0);   /* never block on a stalled USB host */
#endif
#endif

    port = uart_by_num((uint8_t)G431_UART_NUM);
    if (port != nullptr) {
        g431_begin(port, G431_TX_PIN, G431_RX_PIN);
        g431_link_up(port, (int)G431_UART_NUM);
    }

    delay(50);
    usb_write("\r\n\r\n[esp32] bridge " FW_VERSION " booting\r\n");
    usb_printf("[esp32] chip=%s  USB CDC on boot=%d  free heap=%lu\r\n",
               ESP.getChipModel(), USB_IS_CDC,
               (unsigned long)ESP.getFreeHeap());
    usb_printf("[esp32] G431 link: UART%d  TX=%d RX=%d  %d 8N1\r\n",
               G431_UART_NUM, G431_TX_PIN, G431_RX_PIN, G431_BAUD);

    /* Ask the G431 for its status once, so a working link is visible. */
    {
        char reply[LINE_MAX];
        if (g431_ask("?", reply, sizeof(reply), 400u)) {
            usb_printf("[esp32] G431 is alive: \"%s\"\r\n", reply);
        } else {
            usb_write("[esp32] no reply from G431 on the configured UART.\r\n");
            probe_all();
        }
    }
    usb_write("[esp32] ready - type !help for commands\r\n");
}

/* When the link looks down, try to bring it back. The G431 may have been
 * reset (brown-out, firmware flash) while the ESP32 kept running. */
static void link_watchdog(void)
{
    static uint32_t next_try_ms;

    if ((int32_t)(millis() - next_try_ms) < 0) return;
    next_try_ms = millis() + LINK_RETRY_MS;

    if (g431 != nullptr) {
        if (last_reply_ms != 0u && (uint32_t)(millis() - last_reply_ms) > 15000u) {
            usb_write("[bridge] link silent for >15 s, re-checking\r\n");
            g431 = nullptr;
        } else {
            return;
        }
    }
    {
        HardwareSerial *port = uart_by_num((uint8_t)G431_UART_NUM);
        char reply[LINE_MAX];
        g431_begin(port, G431_TX_PIN, G431_RX_PIN);
        delay(20);
        if (g431_ask("?", reply, sizeof(reply), 300u)) {
            g431_link_up(port, (int)G431_UART_NUM);
            usb_printf("[bridge] link recovered: \"%s\"\r\n", reply);
        } else {
            /* Do not end() the UART: a read that lands between end() and the
               next begin() makes ESP-IDF log "uart driver error". */
            g431 = nullptr;
        }
    }
}

void loop(void)
{
    while (Serial.available() > 0) {
        int v = Serial.read();
        if (v < 0) break;
        usb_handle_byte((uint8_t)v);
    }
    g431_poll(nullptr, 0u);
    link_watchdog();
    delay(1);
}
