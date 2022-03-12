#!/bin/bash

rm -rf ./build
mkdir build
export PICO_SDK_PATH=~/pico/pico-sdk
cd ~/robotics/sauvc-2022/firmware/pwm-test/build
cmake ..
make -j6
