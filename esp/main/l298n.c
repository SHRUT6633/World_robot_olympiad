#include "l298n.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

/* Tag used for ESP_LOG* messages to identify this module. */
static const char *TAG = "L298N";

/* Pin assignments for the L298N motor driver (SINGLE MOTOR configuration).
 *
 * All four wheels are driven by ONE motor via a chain/gear drivetrain.
 * There is NO second motor — this is a single-channel L298N setup.
 *
 * ENA (GPIO 11) : PWM enable pin – duty cycle controls motor speed.
 * IN1 (GPIO 8)  : Direction input A.
 * IN2 (GPIO 9)  : Direction input B.
 *
 * WRO Rule 11.3 compliance:
 *   - Exactly one steering actuator (one servo for all 4 wheels via linkage).
 *   - Exactly one drive motor (single L298N channel for all 4 wheels).
 *   - No electronic differential (Rule 11.5) — mechanical AWD only.
 *
 * Changing these pins requires updating the board wiring and the
 * GPIO_OUTPUT_PIN_SEL mask below.
 */
#define PIN_ENA   11
#define PIN_IN1   8
#define PIN_IN2   9

/* Bit mask selecting IN1 and IN2 as output GPIOs.
 * (1ULL<<8) | (1ULL<<9)  – ENA is driven by LEDC, not by direct GPIO.
 */
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<PIN_IN1)|(1ULL<<PIN_IN2))

/* l298n_init
 * Configures the L298N driver hardware.
 *
 * Steps performed:
 *   1. Configure IN1 and IN2 as standard push-pull outputs,
 *      initialised LOW (motor coast/brake depending on ENA).
 *   2. Set up LEDC timer 1 at 20 kHz with 10-bit resolution.
 *      Higher frequency reduces audible coil whine but increases
 *      switching losses. 20 kHz is above human hearing.
 *   3. Attach LEDC channel 1 to ENA (GPIO 11) with duty = 0,
 *      so the motor is stopped after init.
 */
void l298n_init(void) {
    /* --- GPIO setup for IN1, IN2 --- */
    gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,  /* only IN1 and IN2 */
        .mode = GPIO_MODE_OUTPUT,             /* push-pull output */
        .pull_up_en = GPIO_PULLUP_DISABLE,    /* no internal pull-up */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,/* no internal pull-down */
        .intr_type = GPIO_INTR_DISABLE,       /* no interrupts needed */
    };
    gpio_config(&io_conf);

    /* Initialise outputs LOW – motor not moving. */
    gpio_set_level(PIN_IN1, 0);
    gpio_set_level(PIN_IN2, 0);

    /* --- LEDC timer config for PWM (shared with ENA) --- */
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,    /* low-speed mode is sufficient */
        .timer_num = LEDC_TIMER_1,            /* use timer 1 */
        .duty_resolution = 10,                /* 10 bits → duty 0..1023 */
        .freq_hz = 20000,                     /* 20 kHz PWM frequency */
        .clk_cfg = LEDC_AUTO_CLK,            /* auto-select clock source */
    };
    ledc_timer_config(&timer);

    /* --- LEDC channel config for ENA pin --- */
    ledc_channel_config_t chan = {
        .gpio_num = PIN_ENA,                 /* ENA pin (GPIO 11) */
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,            /* use channel 1 */
        .timer_sel = LEDC_TIMER_1,            /* bind to timer 1 */
        .duty = 0,                            /* start at 0 % duty */
        .hpoint = 0,                          /* phase offset */
    };
    ledc_channel_config(&chan);

    ESP_LOGI(TAG, "L298N initialized (ENA=GPIO11, IN1=GPIO8, IN2=GPIO9)");
}

/* l298n_set_motor
 * Drives the motor at a given speed and direction.
 *
 * speed_pct : signed speed request.
 *   If negative, the absolute value is used and direction is forced
 *   to reverse. This avoids accidental forward movement when a
 *   caller passes a negative value with forward=true.
 * forward   : true  → IN1=1, IN2=0 (forward).
 *             false → IN1=0, IN2=1 (reverse).
 *
 * The duty cycle is calculated as:
 *   duty = speed_pct * 1023 / 100
 * giving a linear mapping from 0 % (duty=0) to 100 % (duty=1023).
 *
 * Changing the L298N wiring or using a different motor may require
 * swapping the IN1/IN2 polarity or adjusting the resolution/frequency.
 *
 * Note: The L298N has a minimum voltage threshold (~1.4 V per darlington
 * pair); very low duty cycles may not produce enough voltage to
 * overcome motor inertia.
 */
void l298n_set_motor(int speed_pct, bool forward) {
    /* Normalise negative speed to reverse direction. */
    if (speed_pct < 0) {
        speed_pct = -speed_pct;   /* use absolute value */
        forward = false;
    }
    /* Clamp to valid range. */
    if (speed_pct > 100) speed_pct = 100;

    /* Set direction via IN1/IN2.
     * L298N truth table:
     *   IN1=1, IN2=0 → forward (A-B)
     *   IN1=0, IN2=1 → reverse (B-A)
     *   IN1=0, IN2=0 → coast (motor free)
     *   IN1=1, IN2=1 → brake (motor shorted)
     * The ENA pin must also be high (via PWM) for the motor to spin.
     */
    gpio_set_level(PIN_IN1, forward ? 1 : 0);
    gpio_set_level(PIN_IN2, forward ? 0 : 1);

    /* Map percentage [0..100] to 10-bit duty [0..1023]. */
    uint32_t duty = (uint32_t)speed_pct * 1023 / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    ESP_LOGD(TAG, "L298N: speed=%d%% forward=%s", speed_pct, forward ? "yes" : "no");
}
