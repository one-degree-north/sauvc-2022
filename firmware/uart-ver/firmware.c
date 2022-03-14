#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"


// convenience things...
#define u8              uint8_t
#define u16             uint16_t
#define u32             uint32_t
#define i8              uint8_t
#define i16             uint16_t
#define i32             uint32_t

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
const u8 thrusterPins[] = {THRUSTER_ONE, THRUSTER_TWO, THRUSTER_THREE,
                              THRUSTER_FOUR, THRUSTER_FIVE, THRUSTER_SIX};
const u8 servoPins[] = {SERVO_LEFT, SERVO_RIGHT};
const u8 killSwitch = KILLSWITCH;
const u8 numThrusters = 6;
const u8 numServos = 2;

/*** SECTION: UTILITIES ***/

bool assertRange(i32 value, i32 min, i32 max) {
    return min <= value && value <= max;
}

/*** SECTION: COMMUNICATIONS ***/

#define MAX_UART_QUEUE      45
#define PACKET_HEADER       0x4d
#define PACKET_FOOTER       0x5c

u8 queue[MAX_UART_QUEUE];
u8* queuePtr = queue;
u8 lrc = 0x00;
u8 packetLength = 0;

void setupUART() {
    uart_init(uart0, 230400);

    // UART0 on Feather RP2040 is bound to GPIO0, GPIO1
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
}

void resetQueue() {
    // too many bytes! reset the queue
    queuePtr = queue;
    lrc = 0x00;
    packetLength = 0;
}

// read the byte c into queue
void readByte(u8 c) {
    *queuePtr = c;
    queuePtr++;
    lrc += c;
    lrc &= 0xFF;
}

void parsePacket() {
    // by this point, the packet's structure must be valid.
    // but make sure header is correct for sanity's sake
    if (queue[0] != PACKET_HEADER) return;

    u8 cmd = queue[1];
    u8 param = queue[2];
    u8 len = queue[3];
    u8* data = queue + 4;

    // temporary
    return;
}

// parse the byte c as part of the packet
void parseByte(u8 c) {
    u8 offset = (queuePtr - queue) / sizeof(u8);
    if (offset >= MAX_UART_QUEUE) {
        resetQueue();
        return;
    }

    if (offset == 3) {
        // byte 3 defines final packet length, or 6+c
        if (packetLength > 16) {
            resetQueue();
            return;
        }
        packetLength = 6 + c;
        readByte(c);
        return;
    }

    if (offset == packetLength - 2) {
        // byte len-2: longitudinal redundancy check
        lrc = ((lrc ^ 0xFF) + 1) & 0xFF;
        if (lrc != c) {
            resetQueue();
            return;
        }
        return;
    }

    if (offset == packetLength - 1) {
        // byte len-1: footer
        if (c == PACKET_FOOTER) {
            parsePacket();
        }
        resetQueue();
        return;
    }

    // default case (all other bytes)
    readByte(c);
}

void readUART() {
    if (uart_is_readable(uart0)) {
        u8 c = uart_getc(uart0);

        if (queuePtr == queue) {
            // no command is currently being written
            // only start writing command if header is detected
            if (c == PACKET_HEADER) {
                *queuePtr = c;
                queuePtr++;
                lrc = c;
            }
        } else {
            // commands are currently being written
            parseByte(c);
        }
    }
}

/*** SECTION: THRUSTERS AND SERVOS ***/

void initOneShot125(uint pin) {
    // initialize the pin
    gpio_init(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);
    u8 slice = pwm_gpio_to_slice_num(pin);
    u8 channel = pwm_gpio_to_channel(pin);

    // configure PWM
    pwm_set_phase_correct(slice, false);
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
    u8 slice = pwm_gpio_to_slice_num(pin);
    u8 channel = pwm_gpio_to_channel(pin);

    // configure PWM
    pwm_set_phase_correct(slice, false);
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

void setServo(uint servo, uint level) {
    // level: [1000, 2000]
    if (!assertRange(level, 1000, 2000)) return;

    u8 slice = pwm_gpio_to_slice_num(servoPins[servo]);
    u8 channel = pwm_gpio_to_channel(servoPins[servo]);

    pwm_set_chan_level(slice, channel, level);
}

void setThruster(uint thruster, uint level) {
    // level: [0, 1000]
    if (!assertRange(level, 0, 1000)) return;

    u8 slice = pwm_gpio_to_slice_num(thrusterPins[thruster]);
    u8 channel = pwm_gpio_to_channel(thrusterPins[thruster]);

    pwm_set_chan_level(slice, channel, level + 1000);
}

void setupOutputs() {
    for (int i = 0; i < numThrusters; ++i) {
        initOneShot125(thrusterPins[i]);
    }
    for (int i = 0; i < numServos; ++i) {
        initServo(servoPins[i]);
    }
}

/*** SECTION: ANALOG INPUTS ***/

void initKillSwitch(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);

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

/*** SECTION: COMMANDS ***/


/*** SECTION: MAIN ***/

void setup() {
    // before we do ANYTHING, set the system clock to 96MHz
    set_sys_clock_khz(96000, false);
    setupOutputs();
    initKillSwitch(killSwitch);
    initVoltSensor();
    setupUART();
}


void loop() {
    readUART();
}

int main() {
    setup();

    while(true) {
        loop();
    }

    return 0;
}



