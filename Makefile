APPLICATION = mechaduino_firmware
RIOTBASE ?= /home/seyboman/riot-ros2-seyboman-master-ws/install/RIOT
BOARD ?= native
QUIET ?= 1
WERROR ?= 0
#SRC += main.c
CFLAGS += -DROS_PACKAGE_NAME=\"mechaduino_firmware\"
#CFLAGS += '-DETHOS_UART=UART_DEV(1)'
CFLAGS += '-DSTDIO_UART_DEV=UART_DEV(1)'
CFLAGS += -DTHREAD_STACKSIZE_MAIN=\(2*THREAD_STACKSIZE_DEFAULT+THREAD_EXTRA_STACKSIZE_PRINTF\)
CXXEXFLAGS += -fno-exceptions -fno-rtti -std=c++11
include /home/seyboman/riot-ros2-seyboman-master-ws/install/mechaduino_firmware/Makefile.include
include $(RIOTBASE)/Makefile.include
