#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include <stdio.h>

// convenience things...
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define u64 uint64_t
#define i8 int8_t
#define i16 int16_t
#define i32 int32_t
#define i64 int64_t

#ifndef MIN
#define MIN(a, b)       (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b)       (((a) > (b)) ? (a) : (b))
#endif

#ifndef ABS
#define ABS(a)          ((a) < 0 ? -(a) : (a))
#endif

// define GPIO pins
#define THRUSTER_ONE 7
#define THRUSTER_TWO 8
#define THRUSTER_THREE 9
#define THRUSTER_FOUR 10
#define THRUSTER_FIVE 11
#define THRUSTER_SIX 12

#define MAX_DELTA_POS 2000 // delta us per second

#define SERVO_LEFT 24
#define SERVO_RIGHT 25

#define RED_LED 18
#define GREEN_LED 19

// inputs
#define KILLSWITCH 26
#define VOLTAGE_SENSOR 27
#define DEPTH_SENSOR 28

// thruster configuration: OneShot42
// MultiShot seems to unfortunately be too fast for PWM driver :sob:
#define PERIOD_MS 42
#define PWM_CLOCK_MHZ 24

// servo configuration
#define SERVO_MIN 1000
#define SERVO_MID 1500
#define SERVO_MAX 2000

// BNO055 configuration : TODO
#define I2C_PORT i2c1
#define BNO055_ADDR 0x28

#define NUM_THRUSTERS 6
#define NUM_SERVOS 2

// serial configuration
#define MAX_UART_QUEUE 45
#define PACKET_HEADER 0x4d
#define PACKET_FOOTER 0xd4

#define OUT_PACKET_HEADER 0x5c
#define OUT_PACKET_FOOTER 0xc5

// alias for constants
const u8 thrusterPins[] =   {THRUSTER_ONE, THRUSTER_TWO, THRUSTER_THREE,
                             THRUSTER_FOUR, THRUSTER_FIVE, THRUSTER_SIX};
const u8 servoPins[] = {SERVO_LEFT, SERVO_RIGHT};
const u8 killSwitch = KILLSWITCH;
const u8 numThrusters = NUM_THRUSTERS;
const u8 numServos = NUM_SERVOS;

// global finite state machine
#define STATE_STOPPED 0
#define STATE_OPERATIONAL 1

u8 fsm_state = STATE_STOPPED;

/*** SECTION: UTILITIES ***/

bool assertRange(i32 value, i32 min, i32 max) {
    return min <= value && value <= max;
}

u8 LRC (u8* bytes, u8 length) {
    u8 LRC = 0x00;
    for (int i = 0; i < length; ++i) {
        LRC = (LRC + bytes[i]) & 0xFF;
    }
    return ((LRC ^ 0xFF) + 1) & 0xFF;
}

/*** SECTION: COMMAND HEADERS ***/
void cmdTest(u8 len, u8 *data);
void cmdReset(u8 len, u8 *data);
void cmdContinue(u8 len, u8 *data);
void cmdStop(u8 len, u8 *data);
void cmdGetAttr(u8 param, u8 len, u8 *data);

void cmdThruster(u8 param, u8 len, u8 *data);
void cmdThrusterMask(u8 param, u8 len, u8 *data);
void cmdGetThruster(u8 param, u8 len, u8 *data);

void cmdServo(u8 param, u8 len, u8 *data);
void cmdGetServo(u8 param, u8 len, u8 *data);
void cmdGetSensor(u8 param, u8 len, u8 *data);

void cmdSetAutoReport(u8 param, u8 len, u8 *data);

void soft_halt();
void halt();
void reset();

// responses

void retHello();
void retEcho(u8 len, u8 *data);
void retAttr(u8 attr);
void retSuccess(bool success);

void retThruster(u8 idx);
void retAllThrusters();

void retServo(u8 idx);
void retAllServos();

void retFloatData(u8 param, float* data);
void retVector3Data(u8 param, float* x, float* y, float* z);
void retQuaternionData(u8 param, float* w, float* x, float* y, float* z);
void retIntegerData(u8 param, u16 data);

/*** SECTION: COMMUNICATIONS ***/

u8 queue[MAX_UART_QUEUE];
u8 *queuePtr = queue;
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
    if (queue[0] != PACKET_HEADER)
        return;

    u8 cmd = queue[1];
    u8 param = queue[2];
    u8 len = queue[3];
    u8 *data = queue + 4;

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
        packetLength = 6 + c;
        readByte(c);
        if (packetLength > 18) {
            resetQueue();
            return;
        }
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

/*** SECTION: FINITE STATE MACHINE & RESET ***/

void setupLEDs() {
    gpio_init(GREEN_LED);
    gpio_init(RED_LED);
    gpio_set_dir(GREEN_LED, GPIO_OUT);
    gpio_set_dir(RED_LED, GPIO_OUT);
    gpio_put(GREEN_LED, 0);
    gpio_put(RED_LED, 0);
}

void updateLEDs() {
    if (fsm_state == STATE_OPERATIONAL) {
        gpio_put(GREEN_LED, 1);
        gpio_put(RED_LED, 0);
    } else {
        gpio_put(GREEN_LED, 0);
        gpio_put(RED_LED, 1);
    }
}

bool fsm_stop() {
    if (fsm_state == STATE_OPERATIONAL)
        soft_halt();

    updateLEDs();
    return true;
}

bool fsm_reset() {
    if (fsm_state == STATE_OPERATIONAL)
        halt();
    reset();

    updateLEDs();
    return true;
}

bool fsm_continue() {
    if (fsm_state == STATE_STOPPED)
        fsm_state = STATE_OPERATIONAL;

    updateLEDs();
    return true;
}

/*** SECTION: THRUSTERS AND SERVOS ***/

u16 thrusterPos[6];
u16 targetThrusterPos[6];
u64 prevThrusterLoopMicroseconds;
u16 servoPos[2];

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
    // 96MHz core divided by 12 results in 8MHz, divided by 3999+1 results in 2kHz
    // loop 1000/4000 * 500us = 125us 1500/4000 * 500us = 192us 2000/4000 * 500us
    // = 250us
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
    // 96MHz core divided by 96 results in 1MHz, divided by 19999+1 results in
    // 50Hz loop 1000/20000 * 20000us = 1000us 1500/20000 * 20000us = 1500us
    // 2000/20000 * 20000us = 2000us
    pwm_config_set_clkdiv_int(&config, 96);
    pwm_init(slice, &config, true);
    pwm_set_wrap(slice, 19999);
    pwm_set_chan_level(slice, channel, 1500);
}

void setServo(uint servo, uint level) {
    servoPos[servo] = level;

    // level: [1000, 2000]
    if (!assertRange(level, 1000, 2000))
        return;

    u8 slice = pwm_gpio_to_slice_num(servoPins[servo]);
    u8 channel = pwm_gpio_to_channel(servoPins[servo]);

    pwm_set_chan_level(slice, channel, level);
}

void runThruster(uint thruster, uint level) {
    // level: [0, 1000]
    if (!assertRange(level, 0, 1000))
        return;

    u8 slice = pwm_gpio_to_slice_num(thrusterPins[thruster]);
    u8 channel = pwm_gpio_to_channel(thrusterPins[thruster]);

    pwm_set_chan_level(slice, channel, level + 1000);
}

void setThrusterTarget(uint thruster, uint level) {
    targetThrusterPos[thruster] = level;
}

void setThruster(uint thruster, uint level) { thrusterPos[thruster] = level; }

void setupOutputs() {
    for (int i = 0; i < numThrusters; ++i) {
        initOneShot125(thrusterPins[i]);
        thrusterPos[i] = 1500;
        targetThrusterPos[i] = 1500;
    }
    for (int i = 0; i < numServos; ++i) {
        initServo(servoPins[i]);
        servoPos[i] = 1500;
    }

    prevThrusterLoopMicroseconds = to_us_since_boot(get_absolute_time());
}

void loopOutputs() {
    u64 now = to_us_since_boot(get_absolute_time());
    u64 elapsed_us = prevThrusterLoopMicroseconds - now;

    // set each output to current value
    for (int i = 0; i < numThrusters; ++i) {
        runThruster(i, thrusterPos[i]);
    }
    // lerp each output to new value (closer to target)
    for (int i = 0; i < numThrusters; ++i) {
        u16 targetDelta = ABS(targetThrusterPos[i] - thrusterPos[i]);
        i8 sign = targetThrusterPos[i] - thrusterPos[i] > 0 ? 1 : -1;
        u16 maxDelta = MIN(MAX_DELTA_POS * elapsed_us / 1000000, 0xFFFF);

        i16 movement = MIN(targetDelta, maxDelta) * sign;
        thrusterPos[i] += movement;
    }

    // set each servo to current value
    for (int i = 0; i < numServos; ++i) {
        setServo(i, servoPos[i]);
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
    adc_select_input(VOLTAGE_SENSOR - 26);
    u16 val = adc_read();
    // conversion factor should be 7.8 * AREF, divide by 2^12 for unit conversion
    // factor
    const float conversionFactor = 7.8 * 3.3 / (1 << 12);
    return val * conversionFactor;
}

void initDepthSensor() {
    adc_init();
    // does all our gpio_... work for us
    adc_gpio_init(DEPTH_SENSOR);
    // select input requires ADC channel, 0..3 match 26..29
    adc_select_input(DEPTH_SENSOR - 26);
}

u16 getDepth() {
    adc_select_input(DEPTH_SENSOR - 26);
    u16 val = adc_read();
    // honestly just return the absolute value, we can calibrate in software
    return val;
}

/*** SECTION: COMMANDS ***/
void cmdTest(u8 len, u8 *data) {
    if (len == 0)
        retHello();
    else
        retEcho(len, data);
}

void cmdReset(u8 len, u8 *data) { retSuccess(fsm_reset()); }

void cmdContinue(u8 len, u8 *data) { retSuccess(fsm_continue()); }

void cmdStop(u8 len, u8 *data) { retSuccess(fsm_stop()); }

void cmdGetAttr(u8 param, u8 len, u8 *data) {
    if (param == 0)
        retAttr(fsm_state);
    else
        retSuccess(false);
}

void cmdThruster(u8 param, u8 len, u8 *data) {
    if ((param < 0x10 || param > 0x15) && param != 0x1F)
        return retSuccess(false);

    if (len == 2) {
        u16 val = data[0] * 0xFF + data[1];
        setThrusterTarget(param - 0x10, val);
    }

    if (len == 12) {
        for (int i = 0; i < 6; ++i) {
            u16 val = data[i * 2] * 0xFF + data[i * 2 + 1];
            setThrusterTarget(i, val);
        }
    }
}

void cmdThrusterMask(u8 param, u8 len, u8 *data) {
    if (param >> 6 != 0b11)
        return retSuccess(false);

    if (len == 2) {
        u8 param_ptr = param;
        u16 val = data[0] * 0xFF + data[1];

        for (int i = 0; i < 6; ++i) {
            param_ptr >>= 1;
            if (param_ptr & 0b1) {
                setThrusterTarget(i, val);
            }
        }
    }
}

void cmdGetThruster(u8 param, u8 len, u8 *data) {
    if ((param < 0x10 || param > 0x15) && param != 0x1F)
        return retSuccess(false);

    if (param == 0x1F) return retAllThrusters();
    else return retThruster(param - 0x10);
}

void cmdServo(u8 param, u8 len, u8 *data) {
    if (param != 0x20 && param != 0x21 && param != 0x25) return retSuccess(false);

    if (len == 2) {
        u16 val = data[0] * 0xFF + data[1];
        setServo(param - 0x20, val);
    }

    if (len == 4) {
        u16 valA = data[0] * 0xFF + data[1];
        u16 valB = data[2] * 0xFF + data[3];
        setServo(0x20, valA);
        setServo(0x21, valB);
    }
}

void cmdGetServo(u8 param, u8 len, u8 *data) {
    if ((param < 0x20 || param > 0x21) && param != 0x2F)
        return retSuccess(false);

    if (param == 0x2F) return retAllServos();
    else return retServo(param - 0x20);
}

void cmdGetSensor(u8 param, u8 len, u8 *data);

void cmdSetAutoReport(u8 param, u8 len, u8 *data);

// responses

void uart_write_data(u8* data, u8 length) {
    // data includes command, param, len.
    u8 lrc = LRC(data, length);

      uart_putc(uart0, OUT_PACKET_HEADER);
    for (int i = 0; i < length; ++i) {
          uart_putc(uart0, data[i]);
    }
      uart_putc(uart0, lrc);
      uart_putc(uart0, OUT_PACKET_FOOTER);
}

void retHello() {
    u8 bytes[] = {
        0x00, 0x00, 5,
        'h', 'e', 'l', 'l', 'o'
    };
    uart_write_data(bytes, 8);
}

void retEcho(u8 len, u8 *data) {
    // incomplete write, therefore we cannot use uart_write_data
      uart_putc(uart0, OUT_PACKET_HEADER);
      uart_putc(uart0, 0x00);
      uart_putc(uart0, 0x00);
      uart_putc(uart0, len);
    for (int i = 0; i < len; ++i) {
          uart_putc(uart0, data[i]);
    }
      uart_putc(uart0, lrc);
      uart_putc(uart0, OUT_PACKET_FOOTER);
}

void retAttr(u8 attr) {
    u8 bytes[] = {
        0x00, 0x0a, 1,
        attr
    };
    uart_write_data(bytes, 4);
}

void retSuccess(bool success) {
    u8 bytes[] = {
        0x0f, 0x0f, 1,
        (u8) success
    };
    uart_write_data(bytes, 4);
}

void retThruster(u8 idx) {
    u8* ptr = (u8*) &thrusterPos[idx];

    u8 bytes[] = {
        0x1a, 0x10 + idx, 2,
        ptr[0], ptr[1]
    };
    uart_write_data(bytes, 5);
}

void retAllThrusters() {
    u8* t0 = (u8*) &thrusterPos[0];
    u8* t1 = (u8*) &thrusterPos[1];
    u8* t2 = (u8*) &thrusterPos[2];
    u8* t3 = (u8*) &thrusterPos[3];
    u8* t4 = (u8*) &thrusterPos[4];
    u8* t5 = (u8*) &thrusterPos[5];

    u8 bytes[] = {
        0x1a, 0x1f, 12,
        t0[0], t0[1], t1[0], t1[1], t2[0], t2[1], t3[0], t3[1], t4[0], t4[1], t5[0], t5[1]
    };
    uart_write_data(bytes, 15);
}

void retServo(u8 idx) {
    u8* ptr = (u8*) &servoPos[idx];

    u8 bytes[] = {
        0x2a, 0x20 + idx, 2,
        ptr[0], ptr[1]
    };
    uart_write_data(bytes, 5);
}

void retAllServos() {
    u8* s0 = (u8*) &servoPos[0];
    u8* s1 = (u8*) &servoPos[1];

    u8 bytes[] = {
        0x2a, 0x2f, 4,
        s0[0], s0[1], s1[0], s1[1]
    };
    uart_write_data(bytes, 7);
}

void retFloatData(u8 param, float* data) {
    u8* ptr = (u8*) data;

    u8 bytes[] = {
        0x33, param, 4,
        ptr[0], ptr[1], ptr[2], ptr[3]
    };
    uart_write_data(bytes, 7);
}

void retVector3Data(u8 param, float* x, float* y, float* z) {
    u8* x_ptr = (u8*) x;
    u8* y_ptr = (u8*) y;
    u8* z_ptr = (u8*) z;

    u8 bytes[] = {
        0x33, param, 12,
        x_ptr[0], x_ptr[1], x_ptr[2], x_ptr[3],
        y_ptr[0], y_ptr[1], y_ptr[2], y_ptr[3],
        z_ptr[0], z_ptr[1], z_ptr[2], z_ptr[3]
    };
    uart_write_data(bytes, 15);
}

void retQuaternionData(u8 param, float* w, float* x, float* y, float* z) {
    u8* w_ptr = (u8*) w;
    u8* x_ptr = (u8*) x;
    u8* y_ptr = (u8*) y;
    u8* z_ptr = (u8*) z;

    u8 bytes[] = {
        0x33, param, 16,
        w_ptr[0], w_ptr[1], w_ptr[2], w_ptr[3],
        x_ptr[0], x_ptr[1], x_ptr[2], x_ptr[3],
        y_ptr[0], y_ptr[1], y_ptr[2], y_ptr[3],
        z_ptr[0], z_ptr[1], z_ptr[2], z_ptr[3]
    };
    uart_write_data(bytes, 19);
}

void retIntegerData(u8 param, u16 data) {
    u8 upper = data / 0xFF;
    u8 lower = data % 0xFF;
    u8 bytes[] = {
        0x33, param, 2,
        upper, lower
    };
    uart_write_data(bytes, 5);
}

/*** SECTION: MAIN ***/

void soft_halt() {
    for (int i = 0; i < numThrusters; ++i) {
        targetThrusterPos[i] = 1500;
    }
    loopOutputs();
}

void halt() {
    for (int i = 0; i < numThrusters; ++i) {
        targetThrusterPos[i] = 1500;
        thrusterPos[i] = 1500;
    }
    loopOutputs();
}

void reset() {
    for (int i = 0; i < numThrusters; ++i) {
        thrusterPos[i] = 1500;
        targetThrusterPos[i] = 1500;
    }
    for (int i = 0; i < numServos; ++i) {
        servoPos[i] = 1500;
    }
    loopOutputs();
}

void setup() {
    // before we do ANYTHING, set the system clock to 96MHz
    set_sys_clock_khz(96000, false);
    setupOutputs();
    initKillSwitch(killSwitch);
    initVoltSensor();
    setupUART();
    setupLEDs();
}

void loop() {
    readUART();
    updateLEDs();
    loopOutputs();
}

int main() {
    bi_decl(bi_program_description("UART version of SAS SAUVC 2022 firmware"));
    bi_decl(bi_1pin_with_name(THRUSTER_ONE, "Thruster 1"));
    bi_decl(bi_1pin_with_name(THRUSTER_TWO, "Thruster 2"));
    bi_decl(bi_1pin_with_name(THRUSTER_THREE, "Thruster 3"));
    bi_decl(bi_1pin_with_name(THRUSTER_FOUR, "Thruster 4"));
    bi_decl(bi_1pin_with_name(THRUSTER_FIVE, "Thruster 5"));
    bi_decl(bi_1pin_with_name(THRUSTER_SIX, "Thruster 6"));
    bi_decl(bi_1pin_with_name(SERVO_LEFT, "Servo 1"));
    bi_decl(bi_1pin_with_name(SERVO_RIGHT, "Servo 2"));
    bi_decl(bi_1pin_with_name(RED_LED, "Red LED"));
    bi_decl(bi_1pin_with_name(GREEN_LED, "Green LED"));
    bi_decl(bi_1pin_with_name(VOLTAGE_SENSOR, "Voltage Sensor"));
    bi_decl(bi_1pin_with_name(DEPTH_SENSOR, "Depth Sensor"));
    bi_decl(bi_1pin_with_name(KILLSWITCH, "Killswitch"));

    setup();

    while (true) {
        loop();
    }

    return 0;
}
