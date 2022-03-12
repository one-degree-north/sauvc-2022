!#/bin/bash

export PICO_SDK_PATH=~/pico/pico-sdk
cd ~/robotics/sauvc-2022/firmware/uart-ver/build
cmake ..
make
