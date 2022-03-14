#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

// define GPIO pins
#define THRUSTER_ONE    7
#define THRUSTER_TWO    8
#define THRUSTER_THREE  9
#define THRUSTER_FOUR   10
#define THRUSTER_FIVE   11
#define THRUSTER_SIX    12

#define SERVO_LEFT      24
#define SERVO_RIGHT     25

// inputs
#define KILLSWITCH      26
#define VOLTAGE_SENSOR  27

// thruster configuration: OneShot42
// MultiShot seems to unfortunately be too fast for PWM driver :sob:
#define PERIOD_MS       42
#define PWM_CLOCK_MHZ   24

// servo configuration
#define SERVO_MIN       1000
#define SERVO_MID       1500
#define SERVO_MAX       2000

// BNO055 configuration : TODO
#define I2C_PORT        i2c1
#define BNO055_ADDR     0x28


// alias for constants
const uint8_t thrusterPins[] = {THRUSTER_ONE, THRUSTER_TWO, THRUSTER_THREE,
                              THRUSTER_FOUR, THRUSTER_FIVE, THRUSTER_SIX};
const uint8_t servoPins[] = {SERVO_LEFT, SERVO_RIGHT};
const uint8_t killSwitch = KILLSWITCH;
const uint8_t numThrusters = 6;
const uint8_t numServos = 2;

// arduino clone lol
int main() {
    setup();

    while(true) {
        loop();
    }

    return 0;
}

void setup() {
    // before we do ANYTHING, set the system clock to 96MHz
    set_sys_clock_khz(96000, false);
    setupOutputs();
    initKillSwitch(killSwitch);

}


void loop() {

}

/*** SECTION: THRUSTERS AND SERVOS ***/

void setupOutputs() {
    for (int i = 0; i < numThrusters; ++i) {
        initOneShot125(thrusterPins[i]);
    }
    for (int i = 0; i < numServos; ++i) {
        initServo(servoPins[i]);
    }
}

void initOneShot125(uint pin) {
    // initialize the pin
    gpio_init(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint8_t slice = pwm_gpio_to_slice_num(pin);
    uint8_t channel = pwm_gpio_to_channel(pin);

    // configure PWM
    pwm_set_phase_correct(slice_one, false);
    pwm_config config = pwm_get_default_config();

    // OneShot125 will run where 1000-2000 map to 125us-250us
    // 96MHz core divided by 12 results in 8MHz, divided by 3999+1 results in 2kHz loop
    // 1000/4000 * 500us = 125us
    // 1500/4000 * 500us = 192us
    // 2000/4000 * 500us = 250us
    pwm_config_set_clkdiv_int(&config, 12);
    pwm_init(slice, &config, true);
    pwm_set_wrap(slice, 3999);
    pwm_set_chan_level(slice, channel, 1500);
}

void initServo(uint pin) {
    // check if pin is on PWM slice 4 or 6, aka GPIO 24,25,28,29
    if (pin != 24 && pin != 25 && pin != 28 && pin != 29) {
        printf("[initServo] ERROR: invalid GPIO pin for PWM init Servo");
        return;
    }

    // initialize the pin
    gpio_init(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint8_t slice = pwm_gpio_to_slice_num(pin);
    uint8_t channel = pwm_gpio_to_channel(pin);

    // configure PWM
    pwm_set_phase_correct(slice_one, false);
    pwm_config config = pwm_get_default_config();

    // OneShot125 will run where 1000-2000 map to 125us-250us
    // 96MHz core divided by 96 results in 1MHz, divided by 19999+1 results in 50Hz loop
    // 1000/20000 * 20000us = 1000us
    // 1500/20000 * 20000us = 1500us
    // 2000/20000 * 20000us = 2000us
    pwm_config_set_clkdiv_int(&config, 96);
    pwm_init(slice, &config, true);
    pwm_set_wrap(slice, 19999);
    pwm_set_chan_level(slice, channel, 1500);
}

/*** SECTION: ANALOG INPUTS ***/

void initKillSwitch(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN)

    // disable pull up/down for good measure, only takes 4 cpu cycles
    gpio_disable_pulls(pin);

    // enable pull up to 3.3v, since we're connecting kill switch to GND
    gpio_pull_up(pin);
    gpio_set_input_enabled(pin, true);
}

bool getKillSwitch() {
    // since we're pulled up to 3.3V, HIGH means not active and LOW means active
    return !gpio_get(killSwitch);
}

void initVoltSensor() {
    adc_init();
    // does all our gpio_... work for us
    adc_gpio_init(VOLTAGE_SENSOR);
    // select input requires ADC channel, 0..3 match 26..29
    adc_select_input(VOLTAGE_SENSOR - 26);
}

float getVoltage() {
    uint16_t val = adc_read();
    // conversion factor should be 7.8 * AREF, divide by 2^12 for unit conversion factor
    const float conversionFactor = 7.8 * 3.3 / (1 << 12);
    return val * conversionFactor;
}






