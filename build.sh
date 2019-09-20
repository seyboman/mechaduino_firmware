#!/bin/bash

make BOARD=mechaduino LINKER_SCRIPT=../RIOT/cpu/sam0_common/ldscripts/samd21j18a_arduino_bootloader.ld PORT=tap0
