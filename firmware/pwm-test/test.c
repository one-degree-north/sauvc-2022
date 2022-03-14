#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define TEST_ONE        20
#define TEST_TWO        24


int main() {
    set_sys_clock_khz(96000, false);

    gpio_init(TEST_ONE);
    gpio_init(TEST_TWO);

    gpio_set_function(TEST_ONE, GPIO_FUNC_PWM);
    gpio_set_function(TEST_TWO, GPIO_FUNC_PWM);

    uint8_t slice_one = pwm_gpio_to_slice_num(TEST_ONE);
    uint8_t channel_one = pwm_gpio_to_channel(TEST_ONE);
    uint8_t slice_two = pwm_gpio_to_slice_num(TEST_TWO);
    uint8_t channel_two = pwm_gpio_to_channel(TEST_TWO);

    //printf("Slice 1: %u", slice_one);
    //printf("Slice 2: %u", slice_two);
    //printf("Channel 1: %u", channel_one);
    //printf("Channel 2: %u", channel_two);

    pwm_config config_one = pwm_get_default_config();
    pwm_config config_two = pwm_get_default_config();

    // slice one goal is 6khz, 62.6us <-> 83.3us out of possible 167us
    // for OneShot42
    pwm_config_set_clkdiv_int(&config_one, 4);
    pwm_init(slice_one, &config_one, true);
    pwm_set_wrap(slice_one, 3999);
    pwm_set_chan_level(slice_one, channel_one, 1500);

    // OneShot125 config: 2khz, 192us <-> 256us out of a possible 512us
    // divisor = 24

    // 500Hz PWM config: 250hz, 1500us <-> 2000us out of a possible 4000us
    // divisor = 192

    // multishot will probably also work, but clkdiv=1 is not a good idea

    // basically servo standard
    // 1000 = min, 1500 = mid, 2000 = max

    // slice two goal is 50hz, 1500us <-> 2000us
    // for servos
    pwm_config_set_clkdiv_int(&config_two, 96);
    pwm_init(slice_two, &config_two, true);
    pwm_set_wrap(slice_two, 19999);
    pwm_set_chan_level(slice_two, channel_two, 1500);

    // servos are easy and straightforward
    // dividing by 96 results in a 1Mhz pulse
    // we can then set periods in microseconds, 1500us = 1500, etc
    // therefore, 1000 is min, 1500 is mid, 2000 is max

    // turn off phase correct
    pwm_set_phase_correct(slice_one, false);
    pwm_set_phase_correct(slice_two, false);


    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_function(LED_PIN, GPIO_OUT);


    while(true) {
        sleep_ms(500);
        gpio_put(LED_PIN, 1);
        pwm_set_chan_level(slice_one, channel_one, 2000);
        pwm_set_chan_level(slice_two, channel_two, 2000);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        pwm_set_chan_level(slice_one, channel_one, 1500);
        pwm_set_chan_level(slice_two, channel_two, 1500);
    }
    return 0;
}

// 500000 -> 322-323Hz
// 644000
