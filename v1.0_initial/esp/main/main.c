#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#include "uart_receiver.h"
#include "packet_validator.h"
#include "crc.h"
#include "command_validator.h"
#include "timeout_detector.h"
#include "watchdog.h"
#include "failsafe.h"
#include "servo_pwm.h"
#include "motor_pwm.h"
#include "tb6612fng.h"
#include "selftest.h"

static const char *TAG = "WRO_4WS";

#define PACKET_HEADER 0xA5
#define PACKET_FOOTER 0x5A
#define LED_GREEN_GPIO 2
#define LED_RED_GPIO   4

typedef enum {
    PKT_MOTOR_COMMAND   = 0x01,
    PKT_SERVO_COMMAND   = 0x02,
    PKT_STEERING_CMD    = 0x03,
    PKT_STATUS_REQ      = 0x04,
    PKT_STATUS_RESP     = 0x05,
    PKT_SELFTEST_REQ    = 0x06,
    PKT_SELFTEST_RESP   = 0x07,
    PKT_EMERGENCY_STOP  = 0xFF,
} packet_type_t;

typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t counter;
    uint8_t msg_type;
    uint8_t length;
    uint8_t payload[24];
    uint16_t crc;
    uint8_t footer;
} uart_packet_t;

typedef enum {
    ESP_STATE_BOOT,
    ESP_STATE_SELFTEST,
    ESP_STATE_READY,
    ESP_STATE_ACTIVE,
    ESP_STATE_ERROR,
    ESP_STATE_FAILSAFE,
} esp_state_t;

typedef struct {
    float servo_angle;
    uint8_t motor_speed;
    uint8_t packet_counter;
    uint32_t last_packet_us;
    bool emergency_stop;
    bool motor_enabled;
    uint32_t uptime_ms;
    uint32_t packets_received;
    uint32_t packets_sent;
    uint32_t crc_errors;
    esp_state_t state;
    esp_selftest_result_t selftest_result;
} app_state_t;

static app_state_t g_state = {0};

static void led_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GREEN_GPIO) | (1ULL << LED_RED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GREEN_GPIO, 0);
    gpio_set_level(LED_RED_GPIO, 0);
}

static void led_green_on(void)  { gpio_set_level(LED_GREEN_GPIO, 1); gpio_set_level(LED_RED_GPIO, 0); }
static void led_red_on(void)    { gpio_set_level(LED_GREEN_GPIO, 0); gpio_set_level(LED_RED_GPIO, 1); }
static void led_off(void)       { gpio_set_level(LED_GREEN_GPIO, 0); gpio_set_level(LED_RED_GPIO, 0); }
static void led_both_on(void)   { gpio_set_level(LED_GREEN_GPIO, 1); gpio_set_level(LED_RED_GPIO, 1); }
static void led_blue(void)      { led_both_on(); }

#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     115200
#define UART_BUF_SIZE      256
#define UART_TX_GPIO       17
#define UART_RX_GPIO       18

static void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    ESP_LOGI(TAG, "UART initialized: %d baud", UART_BAUD_RATE);
}

static void send_packet(uint8_t msg_type, const uint8_t *payload, uint8_t len) {
    uint8_t buf[32];
    int idx = 0;
    buf[idx++] = PACKET_HEADER;
    buf[idx++] = g_state.packet_counter;
    buf[idx++] = msg_type;
    buf[idx++] = len;
    if (payload && len > 0) {
        memcpy(&buf[idx], payload, len);
        idx += len;
    }
    uint16_t crc = crc16(buf, idx);
    buf[idx++] = crc & 0xFF;
    buf[idx++] = (crc >> 8) & 0xFF;
    buf[idx++] = PACKET_FOOTER;
    uart_write_bytes(UART_PORT_NUM, (const char*)buf, idx);
    g_state.packets_sent++;
}

static void send_status_response(void) {
    uint8_t payload[6];
    payload[0] = g_state.selftest_result.uart_ok ? 1 : 0;
    payload[1] = g_state.state;
    payload[2] = g_state.uptime_ms & 0xFF;
    payload[3] = (g_state.uptime_ms >> 8) & 0xFF;
    payload[4] = g_state.packets_received & 0xFF;
    payload[5] = g_state.crc_errors & 0xFF;
    send_packet(PKT_STATUS_RESP, payload, 6);
}

static void send_selftest_response(void) {
    uint8_t payload[8];
    payload[0] = g_state.selftest_result.uart_ok ? 1 : 0;
    payload[1] = g_state.selftest_result.servo_pwm_ok ? 1 : 0;
    payload[2] = g_state.selftest_result.motor_pwm_ok ? 1 : 0;
    payload[3] = g_state.selftest_result.tb6612fng_ok ? 1 : 0;
    payload[4] = g_state.selftest_result.watchdog_ok ? 1 : 0;
    payload[5] = g_state.selftest_result.test_duration_ms & 0xFF;
    payload[6] = (g_state.selftest_result.test_duration_ms >> 8) & 0xFF;
    payload[7] = esp_selftest_all_passed(&g_state.selftest_result) ? 1 : 0;
    send_packet(PKT_SELFTEST_RESP, payload, 8);
}

static void process_packet(uart_packet_t *pkt) {
    if (pkt->msg_type == PKT_EMERGENCY_STOP) {
        g_state.emergency_stop = true;
        g_state.state = ESP_STATE_FAILSAFE;
        motor_set_speed(0);
        servo_set_angle(0);
        led_red_on();
        ESP_LOGW(TAG, "EMERGENCY STOP");
        return;
    }
    if (g_state.emergency_stop) return;

    switch (pkt->msg_type) {
        case PKT_STEERING_CMD: {
            if (pkt->length >= 5) {
                float angle;
                memcpy(&angle, pkt->payload, sizeof(float));
                uint8_t speed = pkt->payload[4];
                g_state.servo_angle = angle;
                g_state.motor_speed = speed;
                servo_set_angle(angle);
                motor_set_speed(speed);
                g_state.motor_enabled = true;
                g_state.state = ESP_STATE_ACTIVE;
            }
            break;
        }
        case PKT_STATUS_REQ:
            send_status_response();
            break;
        case PKT_SELFTEST_REQ:
            send_selftest_response();
            break;
        default:
            break;
    }
}

static void uart_rx_task(void *arg) {
    uint8_t *rx_buf = malloc(UART_BUF_SIZE);
    uint8_t packet_buf[32];
    int packet_idx = 0;
    bool in_packet = false;

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, rx_buf, UART_BUF_SIZE, pdMS_TO_TICKS(10));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uint8_t byte = rx_buf[i];
                if (!in_packet && byte == PACKET_HEADER) {
                    in_packet = true;
                    packet_idx = 0;
                    packet_buf[packet_idx++] = byte;
                } else if (in_packet) {
                    packet_buf[packet_idx++] = byte;
                    if (byte == PACKET_FOOTER && packet_idx >= 8) {
                        uart_packet_t *pkt = (uart_packet_t*)packet_buf;
                        uint16_t calc_crc = crc16(packet_buf, packet_idx - 3);
                        if (calc_crc == pkt->crc) {
                            g_state.packet_counter = pkt->counter;
                            g_state.last_packet_us = esp_timer_get_time();
                            process_packet(pkt);
                            g_state.packets_received++;
                        } else {
                            g_state.crc_errors++;
                            ESP_LOGW(TAG, "CRC error: calc=0x%04X pkt=0x%04X", calc_crc, pkt->crc);
                        }
                        in_packet = false;
                        packet_idx = 0;
                    }
                    if (packet_idx >= (int)sizeof(packet_buf)) {
                        in_packet = false;
                        packet_idx = 0;
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    free(rx_buf);
}

static void timeout_monitor_task(void *arg) {
    const uint64_t timeout_us = 500000;
    while (1) {
        uint64_t now = esp_timer_get_time();
        uint64_t elapsed = now - g_state.last_packet_us;
        if (g_state.last_packet_us > 0 && elapsed > timeout_us && g_state.motor_enabled) {
            ESP_LOGW(TAG, "Comm timeout! Stopping motors.");
            motor_set_speed(0);
            servo_set_angle(0);
            g_state.motor_enabled = false;
            g_state.state = ESP_STATE_FAILSAFE;
            led_red_on();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void watchdog_task(void *arg) {
    while (1) { watchdog_feed(); vTaskDelay(pdMS_TO_TICKS(500)); }
}

static void status_task(void *arg) {
    while (1) {
        g_state.uptime_ms = esp_timer_get_time() / 1000;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void led_indicator_task(void *arg) {
    while (1) {
        switch (g_state.state) {
            case ESP_STATE_BOOT:      led_both_on(); break;
            case ESP_STATE_SELFTEST:  led_blue();    break;
            case ESP_STATE_READY:     led_green_on(); break;
            case ESP_STATE_ACTIVE:    led_green_on(); break;
            case ESP_STATE_ERROR:     led_red_on();  break;
            case ESP_STATE_FAILSAFE:  led_red_on();  break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "WRO 4WS ESP32-S3 v1.0 + Self-Test");
    ESP_LOGI(TAG, "Booting...");

    memset(&g_state, 0, sizeof(g_state));
    g_state.state = ESP_STATE_BOOT;
    g_state.last_packet_us = 0;
    g_state.emergency_stop = false;

    led_init();
    led_both_on();
    uart_init();
    tb6612fng_init();
    servo_pwm_init();
    motor_pwm_init();
    watchdog_init();
    failsafe_init();

    g_state.state = ESP_STATE_SELFTEST;
    esp_selftest_init();
    esp_selftest_run(&g_state.selftest_result);

    if (esp_selftest_all_passed(&g_state.selftest_result)) {
        g_state.state = ESP_STATE_READY;
        led_green_on();
        ESP_LOGI(TAG, "SELF-TEST: ALL PASSED - Green LED ON");
    } else {
        g_state.state = ESP_STATE_ERROR;
        led_red_on();
        ESP_LOGE(TAG, "SELF-TEST: FAILED - Red LED ON");
    }

    xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 10, NULL);
    xTaskCreate(timeout_monitor_task, "timeout_mon", 2048, NULL, 8, NULL);
    xTaskCreate(watchdog_task, "watchdog", 2048, NULL, 9, NULL);
    xTaskCreate(status_task, "status", 2048, NULL, 5, NULL);
    xTaskCreate(led_indicator_task, "led_indicator", 2048, NULL, 6, NULL);

    ESP_LOGI(TAG, "Ready. Waiting for Pi commands...");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
