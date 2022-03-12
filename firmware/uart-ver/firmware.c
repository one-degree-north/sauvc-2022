#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"

// define GPIO pins
#define THRUSTER_ONE    7
#define THRUSTER_TWO    8
#define THRUSTER_THREE  9
#define THRUSTER_FOUR   10
#define THRUSTER_FIVE   11
#define THRUSTER_SIX    12

#define SERVO_LEFT      24
#define SERVO_RIGHT     25

#define KILLSWITCH      26

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


// alias arrays for constants
const uint8_t thrusterPins[] = {THRUSTER_ONE, THRUSTER_TWO, THRUSTER_THREE,
                              THRUSTER_FOUR, THRUSTER_FIVE, THRUSTER_SIX};
const uint8_t servoPins[] = {SERVO_LEFT, SERVO_\RIGHT};
const uint8_t killswitch = KILLSWITCH;


// arduino clone lol
int main() {
    setup();

    while(true) {
        loop();
    }
    return 0;





}

void setup() {
    // debug configuration

}


void loop() {

}

/*** SECTION: THRUSTERS AND SERVOS ***/



void setupOutputs() {
    for (int i = 0; i < 6; ++i) {
        gpio_set_function(
}

