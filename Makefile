#
# 1. take out bat
# 2. echo @temp=OTA | nc rpi-n 8888; echo press ret; read a; echo @temp=xxx | nc rpi-n 8888; ultratemp
# 3. reinsert bat
#
# multiflush in a row:
#    for i in 1 3 4 5 6 7 8; do echo --- $i ---; while ! make esp32-$i ota; do echo repeating...; done; done
#

ifneq ($(findstring esp32-1 ,$(MAKECMDGOALS)),)
    OTA_ADDR = esp32-1
    ultra_temp.ino.cpp_CFLAGS = -DESP32_1 -fpermissive
else ifneq ($(findstring esp32-3 ,$(MAKECMDGOALS)),)
    OTA_ADDR = esp32-3
    ultra_temp.ino.cpp_CFLAGS = -DESP32_3 -fpermissive
else ifneq ($(findstring esp32-4 ,$(MAKECMDGOALS)),)
    OTA_ADDR = esp32-4
    ultra_temp.ino.cpp_CFLAGS = -DESP32_4 -fpermissive
else ifneq ($(findstring esp32-5 ,$(MAKECMDGOALS)),)
    OTA_ADDR = esp32-5
    ultra_temp.ino.cpp_CFLAGS = -DESP32_5 -fpermissive
else ifneq ($(findstring esp32-6 ,$(MAKECMDGOALS)),)
    OTA_ADDR = esp32-6
    ultra_temp.ino.cpp_CFLAGS = -DESP32_6 -fpermissive
else ifneq ($(findstring esp32-7 ,$(MAKECMDGOALS)),)
    OTA_ADDR = esp32-7
    ultra_temp.ino.cpp_CFLAGS = -DESP32_7 -fpermissive
else ifneq ($(findstring esp32-8 ,$(MAKECMDGOALS)),)
    OTA_ADDR = esp32-8
    ultra_temp.ino.cpp_CFLAGS = -DESP32_8 -fpermissive
endif

#ultra_temp.ino.cpp_CFLAGS += -DMCFG_LOCAL

ifeq ($(OTA_ADDR),)
    $(error params wrong or missing: make <machine> <target>)
endif
ifneq ($(findstring ota,$(MAKECMDGOALS)),)
    OTA_HPORT != esp32_time_wait
    $(info OTA_ADDR: $(OTA_ADDR))
    $(info OTA_HPORT: $(OTA_HPORT))
endif

CHIP = esp32
BOARD = esp32
UPLOAD_SPEED = 921600
#1
UPLOAD_PORT = /dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_0255CF15-if00-port0
#2
UPLOAD_PORT = /dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_02EXVH5I-if00-port0

ESP_ROOT = $(HOME)/esp32
LIBS = $(HOME)/Arduino/libraries
include $(HOME)/makeEspArduino/makeEspArduino.mk
BUILD_OPT_H := $(shell touch $(BUILD_DIR)/build_opt.h)  # work around https://github.com/plerup/makeEspArduino/issues/189

esp32-1: FRC
esp32-3: FRC
esp32-4: FRC
esp32-5: FRC
esp32-6: FRC
esp32-7: FRC
esp32-8: FRC

# must recompile it always since last flags were unknown
FRC:
	touch ultra_temp.ino

prune: clean FRC
	rm -frv cscope.out /tmp/mkESP

