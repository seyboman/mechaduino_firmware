#!/bin/bash

make BOARD=arduino-zero LINKER_SCRIPT=/home/seyboman/ros2_riot_ws/install/RIOT/cpu/sam0_common/ldscripts/samd21j18a_arduino_bootloader.ld PORT=tap0
