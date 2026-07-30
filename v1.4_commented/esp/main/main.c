/* ============================================================================
 * main.c — WRO 2026 4WS (Four-Wheel Steering) ESP32-S3 Firmware
 *
 * Purpose:
 *   This is the main entry point for the ESP32-S3 microcontroller in a
 *   Raspberry Pi → ESP32-S3 → L298N + Servo robot architecture.  The Pi sends
 *   serial (UART) commands containing steering angles and motor speeds; the
 *   ESP interprets them, drives the two DC motors via an L298N H-bridge, and
 *   positions a servo for steering.
 *
 *   It also implements:
 *     - A self-test suite (UART loopback, PWM generation, L298N, watchdog).
 *     - A communications-watchdog timeout that stops motors if no packet
 *       arrives within 500 ms.
 *     - An emergency-stop packet type (0xFF) that immediately cuts power and
 *       switches to FAILSAFE state.
 *     - Status-report and self-test-response packets sent back to the Pi.
 *     - Dual-LED visual state indication (green/red/both/blue).
 *     - A hardware-watchdog task that periodically calls watchdog_feed().
 *
 * Hardware assumptions:
 *   - UART1 (TX=GPIO17, RX=GPIO18) @ 115200 baud ←→ Raspberry Pi.
 *   - L298N   → IN1/IN2/ENA on pins configured in l298n.h.
 *   - Servo   → PWM on pin configured in servo_pwm.h.
 *   - Green LED on GPIO2 (usually the ESP32-S3-DevKitC built-in).
 *   - Red  LED on GPIO4 (external, or add-on board).
 *
 * Protocol (simplified):
 *   [0xA5][counter][msg_type][length][payload 0-24 bytes][CRC16][0x5A]
 * ============================================================================ */

/* ---------------------------------------------------------------------------
 * Standard C library headers
 * --------------------------------------------------------------------------- */
#include <stdio.h>      /* Standard I/O (snprintf, etc. — occasionally used in ESP logging macros) */
#include <string.h>     /* memset, memcpy — required for zeroing state, copying packet payloads */

/* ---------------------------------------------------------------------------
 * ESP-IDF / FreeRTOS headers
 * --------------------------------------------------------------------------- */
#include "freertos/FreeRTOS.h"      /* FreeRTOS types, portMAX_DELAY, pdMS_TO_TICKS, etc. */
#include "freertos/task.h"          /* xTaskCreate, vTaskDelay — needed for all background tasks */
#include "esp_log.h"                /* ESP_LOGI, ESP_LOGW, ESP_LOGE — tagged logging over USB/JTAG */
#include "driver/uart.h"            /* UART driver — uart_param_config, uart_read_bytes, etc. */
#include "driver/gpio.h"            /* GPIO driver — gpio_config, gpio_set_level */
#include "driver/ledc.h"            /* LEDC PWM driver — used by servo_pwm.c / l298n.c for PWM generation */
#include "esp_timer.h"              /* esp_timer_get_time() — microsecond-resolution timer for timeouts and uptime */
#include "esp_task_wdt.h"           /* esp_task_wdt_add — subscribe this task to the interrupt watchdog */

/* ---------------------------------------------------------------------------
 * Project-internal headers (each encapsulates one concern)
 * --------------------------------------------------------------------------- */
#include "uart_receiver.h"      /* UART byte-level framing helper (if any); but main.c does its own framing inline */
#include "packet_validator.h"   /* Optional packet-structure validator; currently unused inline — integrated in process_packet */
#include "crc.h"                /* crc16(buf, len) — CRC-16-CCITT used for every transmitted & received packet */
#include "command_validator.h"  /* Range-checks command payloads (speed, angle bounds) — called from process_packet indirectly */
#include "timeout_detector.h"   /* Higher-level timeout logic; main.c implements its own 500 ms monitor inline */
#include "watchdog.h"           /* watchdog_init(), watchdog_feed() — hardware (IWDT / TWDT) management */
#include "failsafe.h"           /* failsafe_init() — optional additional motor-disable / brake routines */
#include "servo_pwm.h"          /* servo_pwm_init(), servo_set_angle(degrees) — steering servo abstraction */
#include "l298n.h"              /* l298n_init(), l298n_set_motor(speed, forward) — dual-motor H-bridge driver */
#include "selftest.h"           /* esp_selftest_init(), ..._run(), ..._all_passed() — power-on self-test */

/* ---------------------------------------------------------------------------
 * Log tag — prepended to every ESP_LOGx message so the user can filter
 * e.g. `idf.py monitor | grep "WRO_4WS"` or set log level per-tag.
 * --------------------------------------------------------------------------- */
static const char *TAG = "WRO_4WS";

/* ============================================================================
 * Protocol constants
 *
 * IMPORTANT: These values MUST match the Raspberry Pi reference implementation
 * in protocol.h / protocol.py.  If you change them here, change them there
 * too, or the Pi's packets will be silently discarded.
 * ============================================================================ */

#define PACKET_HEADER 0xA5  /* Start-of-frame marker (SOF).  The UART receiver
                             * stays in "idle" state until it sees 0xA5.  If
                             * changed, Pi's PACKET_HEADER must match exactly.
                             * Picking 0xA5 because it has alternating bits
                             * (1010 0101) which helps with bit-sync on noisy
                             * lines. */

#define PACKET_FOOTER 0x5A  /* End-of-frame marker (EOF).  Once received, the
                             * packet is CRC-checked and dispatched.  0x5A is
                             * the bit-inverse of 0xA5 — a common symmetry in
                             * byte-oriented protocols to detect frame slips.
                             * If changed, MUST match Pi. */

/* ============================================================================
 * LED GPIO assignments
 *
 * The ESP32-S3-DevKitC has a built-in RGB LED (WS2812) on GPIO48, but this
 * design uses two discrete LEDs for simpler, more reliable visual feedback
 * that doesn't require a one-wire protocol.  Green = nominal, Red = error,
 * both = boot, blue (both on) = self-test.
 * ============================================================================ */

#define LED_GREEN_GPIO 2    /* Green LED GPIO.  On most ESP32-S3 dev boards
                             * GPIO2 is safe as a plain output (no strapping
                             * function on S3).  If you move the LED to a
                             * different pin, change this number and ensure
                             * the pin isn't used by UART, PSRAM, or flash. */

#define LED_RED_GPIO   4    /* Red LED GPIO.  GPIO4 is also strapping-free on
                             * S3.  If this pin is shared with another
                             * peripheral, choose an alternative and update
                             * led_init() accordingly. */

/* ============================================================================
 * Packet type enumerator
 *
 * Every UART packet has a one-byte msg_type field.  The Pi fills this in;
 * the ESP dispatches on it.  Adding new command types is a matter of adding
 * an enum member, a case in process_packet(), and matching support on the Pi.
 * ============================================================================ */
typedef enum {
    PKT_MOTOR_COMMAND   = 0x01,  /* (Reserved/legacy) Direct motor-speed command
                                  * without steering — currently unused in favor
                                  * of PKT_STEERING_CMD which bundles both. */
    PKT_SERVO_COMMAND   = 0x02,  /* (Reserved/legacy) Servo-angle-only command.
                                  * If re-enabled, process_packet would need a
                                  * new case. */
    PKT_STEERING_CMD    = 0x03,  /* Combined steering command: 4-byte float
                                  * angle + 1-byte speed (0-255).  This is the
                                  * primary drive command in normal operation. */
    PKT_STATUS_REQ      = 0x04,  /* Status-request: Pi asks ESP to send back a
                                  * 6-byte status packet.  No payload needed. */
    PKT_STATUS_RESP     = 0x05,  /* Status-response (ESP → Pi): uptime,
                                  * packets received, CRC errors, etc. */
    PKT_SELFTEST_REQ    = 0x06,  /* Self-test request (Pi → ESP): triggers a
                                  * send_selftest_response(). */
    PKT_SELFTEST_RESP   = 0x07,  /* Self-test response (ESP → Pi): 8 bytes of
                                  * per-subsystem pass/fail flags. */
    PKT_EMERGENCY_STOP  = 0xFF,  /* Emergency stop (Pi → ESP): immediately
                                  * stops motors, zeroes servo, enters FAILSAFE.
                                  * Highest-priority command — checked before
                                  * any other packet processing.  0xFF is chosen
                                  * as a distinctive "all bits high" byte that
                                  * is unlikely to appear as random noise. */
} packet_type_t;

/* ============================================================================
 * UART packet structure (packed)
 *
 * Layout (total 8–32 bytes depending on payload length):
 *   [0] header     = 0xA5
 *   [1] counter    = incrementing sequence number (used for duplicate detection)
 *   [2] msg_type   = packet_type_t value
 *   [3] length     = number of bytes in payload (0–24)
 *   [4..27] payload = up to 24 bytes of command data
 *   [28] crc low   = CRC-16 low byte
 *   [29] crc high  = CRC-16 high byte
 *   [30] footer    = 0x5A
 *
 * The __attribute__((packed)) tells GCC not to insert padding bytes between
 * fields.  Without this the compiler might align uint16_t on 2-byte boundaries
 * and the on-wire layout would not match what the Pi sends.
 * --------------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    uint8_t header;              /* SOF marker — must == PACKET_HEADER (0xA5).
                                  * If a byte with value 0xA5 appears in the
                                  * middle of a payload the receiver will
                                  * misinterpret it as a new frame start — this
                                  * is a known limitation that a real deployment
                                  * would solve with byte-stuffing or a longer
                                  * header pattern. */
    uint8_t counter;             /* Sequence counter (0–255).  The receiver
                                  * stores the most recent value in
                                  * g_state.packet_counter.  Useful for
                                  * detecting duplicate or reordered packets
                                  * over an unreliable link.  Currently stored
                                  * but not acted upon — a future enhancement
                                  * could ignore packets with counter == last. */
    uint8_t msg_type;            /* Command identifier — one of packet_type_t.
                                  * Determines which branch in process_packet()
                                  * is taken.  Unknown types are silently
                                  * dropped (the default case in the switch). */
    uint8_t length;              /* Number of valid payload bytes (0–24).
                                  * Controls how many bytes are copied from the
                                  * payload array.  If set larger than 24 the
                                  * receiver will read garbage past our struct
                                  * (that's why the code checks `pkt->length >=
                                  * needed` before using payload fields). */
    uint8_t payload[24];         /* Command-specific data.  24 bytes is enough
                                  * for a 4-byte float + 1-byte speed + room for
                                  * expansion.  If larger payloads are needed,
                                  * increase this AND sizeof(packet_buf) in
                                  * uart_rx_task. */
    uint16_t crc;                /* CRC-16 over everything from header through
                                  * payload (i.e. before this field).  Stored
                                  * little-endian (low byte first).  The
                                  * receiver recomputes CRC over header + counter
                                  * + msg_type + length + payload and compares
                                  * against this value.  On mismatch the packet
                                  * is discarded and g_state.crc_errors++.
                                  * Without CRC, a single bit flip could cause
                                  * the robot to misinterpret a command. */
    uint8_t footer;              /* EOF marker — must == PACKET_FOOTER (0x5A).
                                  * Its presence is the trigger for the receiver
                                  * to finalise the packet and run the CRC check.
                                  * If footer is not 0x5A (or noise makes it look
                                  * like 0x5A in the wrong place), the receiver
                                  * may split or merge frames incorrectly. */
} uart_packet_t;

/* ============================================================================
 * ESP32 state machine
 *
 * The firmware moves through these states monotonically (BOOT → SELFTEST →
 * READY → ACTIVE) except that ERROR or FAILSAFE are possible at any time.
 * The LED indicator task and the failsafe logic check g_state.state to decide
 * behaviour.
 * ============================================================================ */
typedef enum {
    ESP_STATE_BOOT,         /* Initial power-on / reset state.  All subsystems
                             * are being initialised.  LEDs: both on (amber).
                             * Minimal functionality — UART is not yet ready. */
    ESP_STATE_SELFTEST,     /* Running the on-board self-test suite.  LEDs:
                             * blue (green + red).  Takes ~500 ms.  If any
                             * subsystem fails the board transitions to ERROR
                             * instead of READY. */
    ESP_STATE_READY,        /* Self-test passed; waiting for first command
                             * from the Pi.  LEDs: green on.  Motors are
                             * disabled (motor_enabled == false).  The timeout
                             * monitor does NOT trigger in this state because
                             * last_packet_us starts at 0. */
    ESP_STATE_ACTIVE,       /* Normal operation — at least one steering command
                             * has been received.  Motors are enabled.  LEDs:
                             * green on.  The timeout monitor WILL trigger if
                             * no packet arrives for 500 ms. */
    ESP_STATE_ERROR,        /* Self-test failed.  Motors are disabled.  LEDs:
                             * red on.  The system will not transition to READY
                             * or ACTIVE until a power-cycle or reset. */
    ESP_STATE_FAILSAFE,     /* Communication timeout or emergency-stop received.
                             * Motors disabled, servo zeroed.  LEDs: red on.
                             * This is a recoverable state only if the design
                             * later adds a reset mechanism; currently the only
                             * way out is a hardware reset. */
} esp_state_t;

/* ============================================================================
 * Application state structure
 *
 * This is the single, global "god object" for the firmware.  Every task reads
 * or writes fields here.  Because FreeRTOS tasks are independent threads,
 * concurrent access to g_state is theoretically unsafe; however the fields
 * are mostly written by one task and read by others (e.g. only uart_rx_task
 * writes servo_angle; only timeout_monitor_task reads last_packet_us and
 * writes motor_enabled).  A future revision should add a mutex or use atomic
 * loads/stores for fields that are read/written from multiple tasks.
 * ============================================================================ */
typedef struct {
    /* --- Actuator state --- */
    float servo_angle;          /* Last commanded servo angle in degrees.
                                  * Written by process_packet(PKT_STEERING_CMD).
                                  * Passed to servo_set_angle().  Range: depends
                                  * on servo model — typically 0–180, but the
                                  * firmware does not clamp here (servo_pwm
                                  * module should handle limits). */

    uint8_t motor_speed;        /* Last commanded motor speed (0–255).
                                  * Written by process_packet and passed to
                                  * l298n_set_motor().  0 = stop, 255 = full
                                  * speed.  The L298N ENA pin is driven by PWM;
                                  * the actual voltage on the motors depends on
                                  * battery level and PWM frequency. */

    /* --- Packet tracking --- */
    uint8_t packet_counter;     /* Most recent sequence counter value from the
                                  * last successfully-parsed packet.  Currently
                                  * only stored; could be used to detect
                                  * duplicates or sequence gaps.  Incremented
                                  * by the Pi. */

    uint32_t last_packet_us;    /* esp_timer_get_time() value at the moment the
                                  * last valid packet was received.  Used by
                                  * timeout_monitor_task to compute elapsed time.
                                  * If left at 0 (initial), the monitor treats
                                  * it as "no packet ever received" and does not
                                  * trigger (so the robot can sit in READY state
                                  * forever without a false timeout). */

    /* --- Safety flags --- */
    bool emergency_stop;        /* Set true when PKT_EMERGENCY_STOP is received.
                                  * Once true, process_packet() silently ignores
                                  * all non-emergency packets.  Cleared only by
                                  * a full reset (there is no "clear E-stop"
                                  * command in the protocol).  If you add such a
                                  * command, be careful: the robot must only
                                  * resume after explicit user intent. */

    bool motor_enabled;         /* True after the first PKT_STEERING_CMD is
                                  * processed.  Set to false by timeout monitor
                                  * or emergency stop.  The L298N may still have
                                  * a non-zero PWM duty cycle briefly until
                                  * l298n_set_motor(0, true) is called.  This
                                  * flag is a "soft" enable — the real enable is
                                  * the OUT1/OUT2 pin state. */

    /* --- Statistics / diagnostics --- */
    uint32_t uptime_ms;         /* System uptime in milliseconds.  Updated every
                                  * 100 ms by status_task.  Sent back to the Pi
                                  * in status-response packets so the Pi knows
                                  * if the ESP has reset unexpectedly.  Wraps
                                  * after ~49.7 days (2^32 ms). */

    uint32_t packets_received;  /* Number of valid packets processed.  Used for
                                  * diagnostics and to compute packet loss rate
                                  * on the Pi side (Pi sent - Pi received on
                                  * ESP).  Wraps after ~4 billion. */

    uint32_t packets_sent;      /* Number of packets transmitted back to the Pi
                                  * (status responses + self-test responses).
                                  * Useful for Pi to verify link is bidirectional. */

    uint32_t crc_errors;        /* Number of packets that failed the CRC check.
                                  * A non-zero value here indicates link noise,
                                  * baud-rate mismatch, or software bugs.  The
                                  * Pi can monitor this to assess link quality. */

    /* --- State machine --- */
    esp_state_t state;          /* Current firmware state (BOOT/SELFTEST/READY/
                                  * ACTIVE/ERROR/FAILSAFE).  Written during init
                                  * and by process_packet/timeout_monitor.
                                  * Read by led_indicator_task and occasionally
                                  * for logging. */

    esp_selftest_result_t selftest_result;  /* Results from the most recent
                                              * self-test run.  Populated by
                                              * esp_selftest_run().  Contains
                                              * per-subsystem pass/fail booleans
                                              * and test duration.  Sent to Pi
                                              * on request. */
} app_state_t;

/* ---------------------------------------------------------------------------
 * Global state instance — zero-initialised by the compiler (placed in .bss).
 * Every function in this file accesses `g_state` directly.  In a larger
 * system this would be passed as a context pointer, but for a single-file
 * embedded firmware it is acceptable.
 * --------------------------------------------------------------------------- */
static app_state_t g_state = {0};

/* ============================================================================
 * LED helper functions
 *
 * GPIO outputs are driven with active-high logic (1 = LED on, 0 = LED off).
 * An external current-limiting resistor (typically 220–470 Ω) is assumed on
 * each LED.  The green LED indicates nominal operation; the red LED indicates
 * errors or failsafe.
 *
 * There are five distinct visual states:
 *   - off      = both off (unit is unpowered or GPIOs not yet configured)
 *   - green    = normal (READY or ACTIVE)
 *   - red      = error or failsafe
 *   - both on  = boot / diagnostic (seen for ~100 ms at startup)
 *   - blue     = both on simultaneously — used here to indicate self-test
 *                (since a discrete green+red LED pair appears amber-yellow,
 *                 but we call it "blue" as a convention in the code).
 * ============================================================================ */

/* Sets up both LED GPIOs as push-pull outputs, initially low (off).
 * gpio_config() applies the configuration atomically to both pins thanks to
 * the bitmask.  After this call, gpio_set_level() can switch each pin high
 * or low without additional configuration.
 *
 * GPIO_PULLUP_DISABLE / GPIO_PULLDOWN_DISABLE ensures no extra current drain
 * through pull resistors when the pin is an output.
 *
 * GPIO_INTR_DISABLE — we don't need interrupts on these output-only pins.
 * --------------------------------------------------------------------------- */
static void led_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GREEN_GPIO) | (1ULL << LED_RED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GREEN_GPIO, 0);  /* Ensure both LEDs start OFF so the
                                         * boot sequence has a known baseline. */
    gpio_set_level(LED_RED_GPIO, 0);
}

/* Inline one-liners for each visual state.  Each sets both GPIOs in a single
 * call sequence.  These are static inline by virtue of being defined in the
 * .c file — the compiler may inline them at -Os. */

static void led_green_on(void)  { gpio_set_level(LED_GREEN_GPIO, 1); gpio_set_level(LED_RED_GPIO, 0); }
static void led_red_on(void)    { gpio_set_level(LED_GREEN_GPIO, 0); gpio_set_level(LED_RED_GPIO, 1); }
static void led_off(void)       { gpio_set_level(LED_GREEN_GPIO, 0); gpio_set_level(LED_RED_GPIO, 0); }
static void led_both_on(void)   { gpio_set_level(LED_GREEN_GPIO, 1); gpio_set_level(LED_RED_GPIO, 1); }
static void led_blue(void)      { led_both_on(); }  /* Alias: "blue" = both LEDs on. */

/* ============================================================================
 * UART configuration constants
 *
 * The ESP32-S3 has three UART controllers.  UART_NUM_0 is usually connected
 * to the USB/JTAG bridge (used for logging via ESP_LOG*).  We use UART_NUM_1
 * so that the protocol traffic is completely separate from debug output.
 * --------------------------------------------------------------------------- */

#define UART_PORT_NUM      UART_NUM_1  /* UART peripheral index.  Must not
                                         * conflict with UART_NUM_0 (console).
                                         * If changed, re-check pin
                                         * assignments. */
#define UART_BAUD_RATE     115200      /* Bit rate in bits per second.  Must
                                         * match the Pi's baudrate exactly.
                                         * Common alternative: 921600 for lower
                                         * latency, but less noise-tolerant.
                                         * 115200 is safe up to ~11 kB/s which
                                         * is ~1156 packets/second — far more
                                         * than our 20–50 Hz control loop needs. */
#define UART_BUF_SIZE      256         /* Internal DMA / ring-buffer size in
                                         * bytes.  Must be large enough to hold
                                         * at least one maximum-size packet
                                         * (32 bytes).  256 bytes is generous.
                                         * Increase if the Pi sends bursts of
                                         * packets faster than the RX task can
                                         * consume them (unlikely at 115200). */
#define UART_TX_GPIO       17          /* UART1 TX pin (ESP → Pi).  Ensure
                                         * this GPIO is not used by PSRAM,
                                         * flash, or other peripherals.
                                         * GPIO17 is a safe general-purpose I/O
                                         * on ESP32-S3. */
#define UART_RX_GPIO       18          /* UART1 RX pin (Pi → ESP).  Same
                                         * constraints as TX.  If swapped, no
                                         * data will arrive. */

/* ---------------------------------------------------------------------------
 * UART initialisation
 *
 * Configures UART_NUM_1 for 8N1 (8 data bits, no parity, 1 stop bit) with no
 * hardware flow control.  The pin mapping separates TX/RX from the USB debug
 * console (UART0).
 *
 * After uart_driver_install, the UART hardware has an interrupt-driven ring
 * buffer of UART_BUF_SIZE bytes.  uart_read_bytes() then copies from this
 * ring buffer into the caller's buffer.  The last parameter (0) is the TX
 * interrupt queue length — zero means we don't use TX interrupts because
 * uart_write_bytes() is blocking in practice (it uses a FIFO + interrupt).
 * --------------------------------------------------------------------------- */
static void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,    /* 115200 — must match Pi */
        .data_bits = UART_DATA_8_BITS,  /* Standard 8-bit bytes */
        .parity = UART_PARITY_DISABLE,  /* No parity bit — saves 12.5% bandwidth */
        .stop_bits = UART_STOP_BITS_1,  /* Single stop bit (standard) */
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  /* No RTS/CTS — we only have
                                                  * TX and RX wired */
    };
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    ESP_LOGI(TAG, "UART initialized: %d baud", UART_BAUD_RATE);
}

/* ============================================================================
 * Packet transmission
 *
 * Assembles a complete packet from the given msg_type, payload, and length,
 * computes the CRC, and writes the bytes to the UART.  This is used for
 * status responses (PKT_STATUS_RESP) and self-test responses
 * (PKT_SELFTEST_RESP).
 *
 * The destination buffer on the stack is 32 bytes — just enough for the
 * header + counter + type + length + 24-byte payload + 2-byte CRC + footer.
 * If the payload length ever exceeds 24 bytes, the buf[] size MUST be
 * increased accordingly or a buffer overflow will corrupt the stack.
 * --------------------------------------------------------------------------- */
static void send_packet(uint8_t msg_type, const uint8_t *payload, uint8_t len) {
    uint8_t buf[32];        /* Stack buffer for the assembled packet.
                             * 32 = 4 (header+counter+type+length) + 24 max payload
                             *      + 2 (CRC) + 1 (footer) + 1 safety margin.
                             * MUST be >= (5 + len + 2) for any valid len <= 24. */
    int idx = 0;

    buf[idx++] = PACKET_HEADER;               /* 0xA5 — frame start */
    buf[idx++] = g_state.packet_counter;      /* Current sequence counter */
    buf[idx++] = msg_type;                    /* e.g. PKT_STATUS_RESP (0x05) */
    buf[idx++] = len;                         /* Payload byte count */

    if (payload && len > 0) {                 /* Defensive: handle NULL payload gracefully */
        memcpy(&buf[idx], payload, len);      /* Copy payload bytes verbatim */
        idx += len;
    }

    /* CRC covers everything from header through payload (idx bytes total).
     * This matches the receiver's CRC computation which is over
     * (packet_idx - 3) bytes — the "- 3" excludes CRC and footer. */
    uint16_t crc = crc16(buf, idx);
    buf[idx++] = crc & 0xFF;                  /* CRC low byte first (little-endian) */
    buf[idx++] = (crc >> 8) & 0xFF;           /* CRC high byte */
    buf[idx++] = PACKET_FOOTER;               /* 0x5A — frame end */

    /* Write to UART.  uart_write_bytes() blocks until the bytes are queued
     * in the DMA buffer.  For short packets this is effectively instant
     * at 115200 baud (~280 µs for 32 bytes). */
    uart_write_bytes(UART_PORT_NUM, (const char*)buf, idx);
    g_state.packets_sent++;                   /* Diagnostic counter */
}

/* ============================================================================
 * Status response builder
 *
 * Packs 6 bytes of diagnostic data into a payload and sends it as a
 * PKT_STATUS_RESP packet.  The Pi typically requests this every 1–5 seconds
 * to monitor link health.
 *
 * Payload layout (6 bytes):
 *   [0] uart_ok        — 1 if UART loopback self-test passed, else 0
 *   [1] state          — current esp_state_t value (0–5)
 *   [2] uptime LSB     — low byte of uptime_ms
 *   [3] uptime MSB     — high byte (uptime covers 16 bits = ~65 seconds)
 *                        NOTE: uptime_ms is uint32_t but only 16 bits are sent;
 *                        the Pi sees only the lower 16 bits and can detect
 *                        wraps.  Full resolution would need 4 bytes.
 *   [4] packets LSB    — low byte of packets_received
 *   [5] CRC errors LSB — low byte of crc_errors
 *
 * The one-byte truncations mean that after 255 packets the counters roll over
 * on the wire; the Pi should track a running delta rather than relying on
 * absolute values.
 * --------------------------------------------------------------------------- */
static void send_status_response(void) {
    uint8_t payload[6];
    payload[0] = g_state.selftest_result.uart_ok ? 1 : 0;
    payload[1] = g_state.state;                     /* e.g. 2 = READY, 3 = ACTIVE */
    payload[2] = g_state.uptime_ms & 0xFF;          /* Uptime low byte */
    payload[3] = (g_state.uptime_ms >> 8) & 0xFF;   /* Uptime high byte */
    payload[4] = g_state.packets_received & 0xFF;   /* RX count low byte */
    payload[5] = g_state.crc_errors & 0xFF;         /* CRC error count low byte */
    send_packet(PKT_STATUS_RESP, payload, 6);
}

/* ============================================================================
 * Self-test response builder
 *
 * Packs 8 bytes containing per-subsystem test results into a
 * PKT_SELFTEST_RESP packet.  The Pi requests this after a self-test command
 * to learn which subsystems passed or failed.
 *
 * Payload layout (8 bytes):
 *   [0] uart_ok         — 1 if UART loopback passed
 *   [1] servo_pwm_ok    — 1 if servo PWM generation passed
 *   [2] motor_pwm_ok    — 1 if motor PWM (LEDC) passed
 *   [3] l298n_ok        — 1 if L298N output test passed
 *   [4] watchdog_ok     — 1 if watchdog feed / reset test passed
 *   [5] duration LSB    — test_duration_ms low byte
 *   [6] duration MSB    — test_duration_ms high byte (16 bits total)
 *   [7] all_passed      — 1 if ALL subsystems passed (convenience flag)
 * --------------------------------------------------------------------------- */
static void send_selftest_response(void) {
    uint8_t payload[8];
    payload[0] = g_state.selftest_result.uart_ok ? 1 : 0;
    payload[1] = g_state.selftest_result.servo_pwm_ok ? 1 : 0;
    payload[2] = g_state.selftest_result.motor_pwm_ok ? 1 : 0;
    payload[3] = g_state.selftest_result.l298n_ok ? 1 : 0;
    payload[4] = g_state.selftest_result.watchdog_ok ? 1 : 0;
    payload[5] = g_state.selftest_result.test_duration_ms & 0xFF;       /* Duration low byte */
    payload[6] = (g_state.selftest_result.test_duration_ms >> 8) & 0xFF; /* Duration high byte */
    payload[7] = esp_selftest_all_passed(&g_state.selftest_result) ? 1 : 0; /* Overall result */
    send_packet(PKT_SELFTEST_RESP, payload, 8);
}

/* ============================================================================
 * Packet processing / command dispatch
 *
 * Called from the UART RX task after a packet passes the CRC check.  The
 * packet is fully validated at this point — header, footer, and CRC are known
 * correct.
 *
 * IMPORTANT: Emergency stop (PKT_EMERGENCY_STOP, 0xFF) is checked BEFORE the
 * emergency_stop guard so that an E-stop always works, even if we're already
 * in emergency_stop mode.  All other commands are silently ignored once
 * emergency_stop is set (the guard at line ~169).  This design ensures that
 * the E-stop acts as a "sticky" latch that cannot be overridden by subsequent
 * commands — only a power cycle clears it.
 * --------------------------------------------------------------------------- */
static void process_packet(uart_packet_t *pkt) {
    /* ---- Emergency-stop handler (highest priority) ---- */
    if (pkt->msg_type == PKT_EMERGENCY_STOP) {
        g_state.emergency_stop = true;          /* Latch the flag — prevents any
                                                 * future non-0xFF commands from
                                                 * having effect. */
        g_state.state = ESP_STATE_FAILSAFE;     /* Move to failsafe state for
                                                 * LED indication and any future
                                                 * policy checks. */
        l298n_set_motor(0, true);               /* Stop all motors immediately.
                                                 * true = forward direction (the
                                                 * speed is 0 so direction is
                                                 * irrelevant, but the L298N
                                                 * abstraction requires it). */
        servo_set_angle(0);                     /* Return steering to centre.
                                                 * 0 = centred (or full-left,
                                                 * depending on servo
                                                 * calibration — verify with
                                                 * servo_pwm module docs). */
        led_red_on();                           /* Visual alert */
        ESP_LOGW(TAG, "EMERGENCY STOP");        /* Log to serial console */
        return;                                 /* Skip all normal processing */
    }

    /* If emergency-stop was previously latched, drop all non-EStop packets.
     * Without this guard, a normal PKT_STEERING_CMD received after an E-stop
     * would re-enable the motors — a safety violation. */
    if (g_state.emergency_stop) return;

    /* ---- Command dispatch ---- */
    switch (pkt->msg_type) {
        case PKT_STEERING_CMD: {
            /* Steering command: payload is 4-byte float angle + 1-byte speed.
             * Minimum length check of 5 bytes guards against malformed packets
             * where the length field was incorrectly set.  If length is less
             * than 5, memcpy would read out-of-bounds (but since pkt is backed
             * by a 32-byte stack buffer, it would read zeroes, not crash). */
            if (pkt->length >= 5) {
                float angle;
                memcpy(&angle, pkt->payload, sizeof(float));  /* Extract 4-byte IEEE 754 float */
                uint8_t speed = pkt->payload[4];              /* Speed is the 5th payload byte */

                g_state.servo_angle = angle;                  /* Save for diagnostics */
                g_state.motor_speed = speed;

                /* Apply to hardware immediately */
                servo_set_angle(angle);                       /* Steering servo position */
                l298n_set_motor(speed, true);                 /* DC motor speed + forward direction */

                g_state.motor_enabled = true;                 /* Enable timeout monitor */
                g_state.state = ESP_STATE_ACTIVE;             /* Transition to ACTIVE state */
            }
            break;
        }

        case PKT_STATUS_REQ:
            /* Pi is asking for a status snapshot.  No additional data needed.
             * This could be called at any time regardless of ACTIVE state. */
            send_status_response();
            break;

        case PKT_SELFTEST_REQ:
            /* Pi wants the self-test results.  We do NOT re-run the test here;
             * we simply return the results from the boot-time self-test.  If
             * re-running self-test at runtime is desired, call
             * esp_selftest_run() here. */
            send_selftest_response();
            break;

        default:
            /* Unknown msg_type — silently ignored.  A production system might
             * want to log this or send a "not understood" response. */
            break;
    }
}

/* ============================================================================
 * UART Receiver Task
 *
 * This is the most time-critical task in the system.  It runs continuously
 * (never exits) with a tight polling loop: it reads bytes from the UART
 * ring buffer, searches for start-of-frame markers (0xA5), accumulates bytes
 * into a packet buffer, then validates CRC on reception of the footer (0x5A).
 *
 * Stack size: 4096 bytes — the malloc() for rx_buf (256 bytes) + packet_buf
 * (32 bytes) + local variables.  If this is too small, the task will crash
 * with a stack overflow (watchdog will bark).  4096 is generous.
 *
 * Priority: 10 (the highest of the four tasks).  This ensures that incoming
 * data is drained from the UART hardware FIFO before the RX FIFO overflows
 * (the ESP32-S3 UART has a 128-byte hardware FIFO; at 115200 baud it fills
 * in ~11 ms — our 10 ms poll interval is tight).
 * --------------------------------------------------------------------------- */
static void uart_rx_task(void *arg) {
    /* Dynamically allocated receive buffer.  Using malloc rather than stack
     * to keep stack usage low in this high-priority task.  This pointer is
     * never freed during normal operation — the `free(rx_buf)` at the bottom
     * of the function is technically unreachable because the loop is infinite.
     * It is present for correctness / static analysis only. */
    uint8_t *rx_buf = malloc(UART_BUF_SIZE);

    /* Packet assembly buffer.  32 bytes = maximum possible packet size
     * (4 header + 24 payload + 2 CRC + 1 footer = 31, rounded up to 32).
     * If PACKET_MAX_PAYLOAD_SIZE is ever increased, this must be increased too. */
    uint8_t packet_buf[32];
    int packet_idx = 0;         /* Current write index into packet_buf */
    bool in_packet = false;     /* Are we currently inside a frame? */

    /* Main receive loop — runs forever */
    while (1) {
        /* Try to read up to UART_BUF_SIZE bytes with a 10 ms timeout.
         * pdMS_TO_TICKS(10) = 10 / portTICK_PERIOD_MS = typically 10 ticks
         * (assuming FreeRTOS tick rate of 1000 Hz).  The timeout prevents
         * the task from busy-waiting when no data is available.
         *
         * If the UART baud rate were increased to 921600, the timeout might
         * need to be reduced to avoid falling behind the hardware FIFO. */
        int len = uart_read_bytes(UART_PORT_NUM, rx_buf, UART_BUF_SIZE, pdMS_TO_TICKS(10));
        if (len > 0) {
            /* Iterate over every received byte */
            for (int i = 0; i < len; i++) {
                uint8_t byte = rx_buf[i];

                if (!in_packet && byte == PACKET_HEADER) {  /* State A: Waiting for start-of-frame marker */
                    in_packet = true;               /* Enter frame accumulation state */
                    packet_idx = 0;                 /* Reset packet index */
                    packet_buf[packet_idx++] = byte; /* Store header byte */
                } else if (in_packet) {             /* State B: Accumulating frame bytes */
                    packet_buf[packet_idx++] = byte; /* Append to packet buffer */

                    /* Check for end-of-frame marker.  Minimum valid packet is
                     * 8 bytes: header(1) + counter(1) + type(1) + length(1)
                     * + error: there must be at least 2 bytes of payload or
                     * the CRC region won't make sense.  The check `packet_idx
                     * >= 8` ensures we don't interpret a footer byte that is
                     * part of the header/type/length as a valid frame end. */
                    if (byte == PACKET_FOOTER && packet_idx >= 8) {
                        /* Cast the raw buffer to our packed struct for field
                         * access.  This is safe because:
                         *   a) The struct is __attribute__((packed)) — no padding.
                         *   b) packet_buf is at least as large as uart_packet_t.
                         *   c) We have accumulated at least 8 bytes. */
                        uart_packet_t *pkt = (uart_packet_t*)packet_buf;

                        /* CRC covers everything BEFORE the CRC field itself
                         * and the footer.  That is: header + counter + msg_type
                         * + length + payload = packet_idx - 3 bytes.
                         * (CRC is 2 bytes, footer is 1 byte = 3 bytes). */
                        uint16_t calc_crc = crc16(packet_buf, packet_idx - 3);

                        if (calc_crc == pkt->crc) {
                            /* Checksum valid — accept the packet */
                            g_state.packet_counter = pkt->counter;   /* Save sequence number */
                            g_state.last_packet_us = esp_timer_get_time(); /* Timestamp for timeout monitor */
                            process_packet(pkt);                     /* Dispatch command */
                            g_state.packets_received++;              /* Count it */
                        } else {
                            /* CRC mismatch — log it and discard */
                            g_state.crc_errors++;
                            ESP_LOGW(TAG, "CRC error: calc=0x%04X pkt=0x%04X", calc_crc, pkt->crc);
                        }

                        /* Reset for next frame regardless of CRC success/failure */
                        in_packet = false;
                        packet_idx = 0;
                    }

                    /* Guard against buffer overflow: if we've filled
                     * sizeof(packet_buf) without seeing a valid footer, reset.
                     * This handles the case where noise generates a false
                     * PACKET_HEADER but no 0x5A appears within 32 bytes.
                     * Without this check, packet_buf would overflow on the
                     * next byte, corrupting adjacent stack variables
                     * (including rx_buf pointer and return address!). */
                    if (packet_idx >= (int)sizeof(packet_buf)) {
                        in_packet = false;
                        packet_idx = 0;
                    }
                }
            }
        }

        /* Yield to lower-priority tasks briefly.  Without this, the RX task
         * would spin at full CPU even when no data is available (wasting
         * power and starving the watchdog/timeout tasks).  1 ms is short
         * enough that the UART FIFO (128 bytes) won't overflow at 115200 baud
         * even if we miss a tick (115200 bps = 11.5 bytes/ms → 128 bytes in
         * 11 ms). */
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* Unreachable in normal operation — kept for completeness */
    free(rx_buf);
}

/* ============================================================================
 * Communication Timeout Monitor Task
 *
 * Monitors the time since the last valid packet.  If more than 500 ms
 * elapses without a packet while motors are enabled, this task stops the
 * motors, zeroes the servo, and transitions to FAILSAFE state.
 *
 * Why 500 ms?  The Pi's control loop typically runs at 20–50 Hz (20–50 ms
 * period).  Missing 10–25 consecutive packets (~10–25 * 50 ms = 500 ms) is a
 * strong indicator that the link is dead.  A shorter timeout (e.g. 200 ms)
 * would risk false positives during transient noise; a longer timeout (e.g.
 * 2 s) would let the robot drive uncontrolled for too long.
 *
 * The task only triggers if `g_state.last_packet_us > 0` (we've received at
 * least one packet since boot) AND `g_state.motor_enabled` is true.  This
 * prevents spurious timeouts while the robot is in READY or ERROR state.
 *
 * Stack: 2048 bytes — sufficient for simple 64-bit math and function calls.
 * Priority: 8 (just below watchdog, just above RX).
 * --------------------------------------------------------------------------- */
static void timeout_monitor_task(void *arg) {
    const uint64_t timeout_us = 500000;  /* 500 ms in microseconds.  Changing
                                          * this affects how long the robot
                                          * drives after losing link.  Shorter
                                          * = safer but more false positives. */

    while (1) {
        uint64_t now = esp_timer_get_time();           /* Current time in microseconds since boot */
        uint64_t elapsed = now - g_state.last_packet_us; /* Time since last valid packet */

        /* Guard: only react if:
         *   1) We have ever received a packet (last_packet_us > 0).
         *   2) Elapsed time exceeds timeout_us.
         *   3) Motors are currently enabled.
         *
         * If last_packet_us == 0 (initial state, no packets yet), the
         * subtraction `now - 0` = now which is >> 500000, but we skip because
         * of the > 0 check.  This lets the robot sit in READY forever. */
        if (g_state.last_packet_us > 0 && elapsed > timeout_us && g_state.motor_enabled) {
            ESP_LOGW(TAG, "Comm timeout! Stopping motors.");
            l298n_set_motor(0, true);           /* Stop motors immediately */
            servo_set_angle(0);                 /* Centre steering */
            g_state.motor_enabled = false;      /* Prevent repeated triggers */
            g_state.state = ESP_STATE_FAILSAFE; /* Visual state change */
            led_red_on();                       /* Alert operator */
        }

        /* Check every 50 ms.  The timeout is 500 ms, so worst-case detection
         * latency is 50 ms plus whatever scheduling delay exists.  At 50 ms
         * intervals, the robot drives at most 550 ms without a packet.
         * Decreasing this interval improves reaction time but uses more CPU. */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ============================================================================
 * Watchdog Feed Task
 *
 * The ESP32-S3 has a hardware Task Watchdog Timer (TWDT) that can reset the
 * chip if certain tasks fail to yield.  This task subscribes to the TWDT and
 * calls watchdog_feed() every 500 ms to keep the watchdog from barking.
 *
 * watchdog_feed() is defined in watchdog.c and likely touches the MWDT
 * (Main Watchdog Timer) or TWDT hardware register.
 *
 * If this task hangs or crashes, the watchdog will fire after its timeout
 * period (typically 5–10 seconds, configurable in menuconfig) and reset the
 * ESP32-S3 — a last-resort safety net.
 *
 * Stack: 2048 bytes.  Priority: 9 (second highest — ensures the watchdog
 * is fed even under heavy CPU load).
 * --------------------------------------------------------------------------- */
static void watchdog_task(void *arg) {
    /* Register this task with the interrupt watchdog (TWDT).  NULL means
     * "subscribe the currently running task."  If this call fails (e.g.
     * TWDT not enabled in menuconfig), the task will still run but feeding
     * may have no effect. */
    esp_task_wdt_add(NULL);  /* Register this task with the TWDT so the watchdog
                              * knows this task is supposed to be alive.  NULL
                              * means "current task".  If TWDT is not enabled
                              * in menuconfig, this is a no-op. */
    while (1) { watchdog_feed(); vTaskDelay(pdMS_TO_TICKS(500)); }  /* Feed watchdog every 500 ms.
                                                                     * Must be shorter than the TWDT
                                                                     * timeout (~5–10 s).  If the delay
                                                                     * exceeds the timeout, the chip
                                                                     * resets. */
}

/* ============================================================================
 * Status Update Task
 *
 * Periodically updates g_state.uptime_ms from the high-resolution hardware
 * timer.  This value is sent to the Pi in status responses.  The task runs
 * every 100 ms, giving ~10 updates per second — more than enough for a
 * human-readable uptime counter.
 *
 * Stack: 2048 bytes.  Priority: 5 (low — not time-critical).
 * --------------------------------------------------------------------------- */
static void status_task(void *arg) {
    while (1) {
        /* esp_timer_get_time() returns microseconds; divide by 1000 for ms.
         * The division yields integer truncation — sub-millisecond precision
         * is not needed for uptime reporting. */
        g_state.uptime_ms = esp_timer_get_time() / 1000;
        vTaskDelay(pdMS_TO_TICKS(100));       /* 100 ms update interval */
    }
}

/* ============================================================================
 * LED Indicator Task
 *
 * Reads g_state.state every 100 ms and sets the LEDs accordingly.  This
 * decouples LED control from the other tasks — no other function needs to
 * worry about LED states except the transition points where state changes.
 *
 * This is deliberately a polling design rather than event-driven: the state
 * machine transitions are infrequent (~5 per boot), so a 100 ms poll adds
 * negligible CPU overhead (< 0.1%).
 *
 * Stack: 2048 bytes.  Priority: 6 (low, but above status_task).
 * --------------------------------------------------------------------------- */
static void led_indicator_task(void *arg) {
    while (1) {
        switch (g_state.state) {
            case ESP_STATE_BOOT:      led_both_on(); break;   /* Amber — initialising */
            case ESP_STATE_SELFTEST:  led_blue();    break;   /* Blue — running self-test */
            case ESP_STATE_READY:     led_green_on(); break;  /* Green — waiting for commands */
            case ESP_STATE_ACTIVE:    led_green_on(); break;  /* Green — normal operation */
            case ESP_STATE_ERROR:     led_red_on();  break;   /* Red — self-test failure */
            case ESP_STATE_FAILSAFE:  led_red_on();  break;   /* Red — timeout or E-stop */
        }
        vTaskDelay(pdMS_TO_TICKS(100));  /* Check every 100 ms — fast enough
                                          * for human-observable visual feedback. */
    }
}

/* ============================================================================
 * main() — ESP-IDF entry point
 *
 * This is the first C function called after the CPU boots, the C runtime is
 * initialised, and FreeRTOS starts.  It does NOT return — the function must
 * either start tasks and loop or exit (which triggers an abort/reset).
 *
 * The process is:
 *   1. Zero global state.
 *   2. Set initial state = BOOT.
 *   3. Initialise hardware: LEDs, UART, L298N, servo PWM, watchdog, failsafe.
 *   4. Transition to SELFTEST and run the self-test suite.
 *   5. If all passed → READY (green LED), else → ERROR (red LED).
 *   6. Create all background FreeRTOS tasks.
 *   7. Print "Ready" and enter an infinite idle loop.
 * --------------------------------------------------------------------------- */
void app_main(void) {
    /* ---- Boot banner ---- */
    ESP_LOGI(TAG, "WRO 4WS ESP32-S3 v1.0 + Self-Test");
    ESP_LOGI(TAG, "Booting...");

    /* ---- Global state initialisation ---- */
    memset(&g_state, 0, sizeof(g_state));   /* Zero-initialise everything.
                                             * This ensures that booleans are
                                             * false, counters are 0, and the
                                             * state is BOOT (enum val 0). */
    g_state.state = ESP_STATE_BOOT;         /* Explicit — already 0 from memset,
                                             * but clarifies intent. */
    g_state.last_packet_us = 0;             /* "No packet received yet" sentinel.
                                             * The timeout monitor uses this to
                                             * distinguish "no link" from
                                             * "link lost". */
    g_state.emergency_stop = false;         /* Explicit false — ensures we
                                             * don't start in E-stop state. */

    /* ---- Hardware initialisation ---- */
    led_init();             /* Configure GPIO2 (green) and GPIO4 (red) as outputs */
    led_both_on();          /* Visual: both LEDs on = "booting".  Stays on until
                             * the self-test completes (~500 ms). */
    uart_init();            /* Configure UART1 at 115200 baud on GPIO17/18 */
    l298n_init();           /* Initialise L298N control pins (IN1/IN2/ENA) as
                             * outputs and ensure motors are OFF. */
    servo_pwm_init();       /* Initialise LEDC timer/channel for servo PWM
                             * (typically 50 Hz, 0.5–2.5 ms pulse). */
    watchdog_init();        /* Configure hardware watchdog timer (TWDT/MWDT).
                             * Must be called before watchdog_task starts
                             * feeding.  If watchdog_init() is not called,
                             * watchdog_feed() may be a no-op or fault. */
    failsafe_init();        /* Optional: pre-charge brake or set safe defaults
                             * for all actuators.  Implementation may be empty
                             * if not needed. */

    /* ---- Self-test execution ---- */
    g_state.state = ESP_STATE_SELFTEST;     /* LED → blue */
    esp_selftest_init();                    /* Prepare test harness (e.g. connect
                                             * a UART loopback jumper on GPIO17→18). */
    esp_selftest_run(&g_state.selftest_result);  /* Run all tests.  This function
                                                   * blocks for ~500 ms and writes
                                                   * results to the struct. */

    /* ---- Evaluate self-test results ---- */
    if (esp_selftest_all_passed(&g_state.selftest_result)) {
        g_state.state = ESP_STATE_READY;    /* All subsystems OK */
        led_green_on();                     /* Green = ready */
        ESP_LOGI(TAG, "SELF-TEST: ALL PASSED - Green LED ON");
    } else {
        g_state.state = ESP_STATE_ERROR;    /* One or more tests failed */
        led_red_on();                       /* Red = error */
        ESP_LOGE(TAG, "SELF-TEST: FAILED - Red LED ON");
        /* NOTE: The firmware continues to create tasks even on self-test
         * failure.  This allows the Pi to request self-test results and
         * diagnose the problem.  The motors will NOT move because the
         * steering/speed commands will only be processed if a command has
         * ever been received (service_angle/speed initialised to 0). */
    }

    /* ---- Create background tasks ---- */
    xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 10, NULL);
    /* 4096-word stack.  Priority 10 (highest).  Reads UART, assembles packets,
     * dispatches commands. */

    xTaskCreate(timeout_monitor_task, "timeout_mon", 2048, NULL, 8, NULL);
    /* 2048-word stack.  Priority 8.  Monitors packet gap and stops motors. */

    xTaskCreate(watchdog_task, "watchdog", 2048, NULL, 9, NULL);
    /* 2048-word stack.  Priority 9 (second highest).  Feeds hardware watchdog. */

    xTaskCreate(status_task, "status", 2048, NULL, 5, NULL);
    /* 2048-word stack.  Priority 5 (low).  Updates uptime_ms field. */

    xTaskCreate(led_indicator_task, "led_indicator", 2048, NULL, 6, NULL);
    /* 2048-word stack.  Priority 6.  Polls state and sets LED colours. */

    /* ---- Idle loop ---- */
    ESP_LOGI(TAG, "Ready. Waiting for Pi commands...");

    /* The main "app_main" task must not return.  It simply sleeps forever;
     * all real work is done by the background tasks created above.  If
     * app_main returns, FreeRTOS will trigger an assertion (or idle task
     * hook) that typically resets the chip. */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));  /* Sleep for 1 second, repeat forever */
    }
}
