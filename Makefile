APPLICATION = mechaduino_firmware
RIOTBASE ?= /home/seyboman/ros2-seyboman_riot_ws/install/RIOT
BOARD ?= native
QUIET ?= 1
WERROR ?= 0
SRC += main.c
CFLAGS += -DROS_PACKAGE_NAME=\"mechaduino_firmware\"
#CFLAGS += '-DETHOS_UART=UART_DEV(1)'
CFLAGS += '-DSTDIO_UART_DEV=UART_DEV(1)'
include /home/seyboman/ros2-seyboman_riot_ws/install/mechaduino_firmware/Makefile.include
include $(RIOTBASE)/Makefile.include
