MODULE_NAME := src/log
ALL_SUB_DIRS := sample src/cmp src/coap src/cota src/dm src/fota src/http src/log src/mqtt src/ota src/platform src/sdk-tests src/shadow src/subdev src/system src/tfs src/tls src/utils
AR := xtensa-lx106-elf-ar
CC := xtensa-lx106-elf-gcc
CFLAGS := -DCMP_ENABLED -DCMP_VIA_MQTT_DIRECT -DDM_ENABLED -DHTTP_COMM_ENABLED -DIOTX_NET_INIT_WITH_PK_EXT -DIOTX_WITHOUT_ITLS -DMQTT_COMM_ENABLED -DMQTT_DIRECT -DMQTT_DIRECT_NOITLS -DMQTT_SHADOW -DOTA_ENABLED -DOTA_SIGNAL_CHANNEL=1 -DSERVICE_OTA_ENABLED -DSUBDEV_VIA_MQTT -fdata-sections -ffunction-sections -mlongcalls
COMP_LIB := libiot_sdk.a
CONFIG_VENDOR := espressif
CROSS_CANDIDATES := AR CC CXX LD OBJCOPY STRIP
CROSS_PREFIX := xtensa-lx106-elf-
CXX := xtensa-lx106-elf-g++
IMPORT_DIR := /home/ytzhitong/github/esp-aliyun-feature-for_esp8266_rtos_sdk_v2.x.x/components/aliyun/iotkit-embedded/src/import
INSTALL_DIR := /home/ytzhitong/github/esp-aliyun-feature-for_esp8266_rtos_sdk_v2.x.x/components/aliyun/iotkit-embedded/.O/usr
INSTALL_LIB_DIR := /home/ytzhitong/github/esp-aliyun-feature-for_esp8266_rtos_sdk_v2.x.x/components/aliyun/iotkit-embedded/.O/usr/lib
KERNEL_DIR := 
LD := xtensa-lx106-elf-ld
LDFLAGS := 
MAKE_ENV_VARS := 
OBJCOPY := xtensa-lx106-elf-objcopy
PACKAGE_DIR := /home/ytzhitong/github/esp-aliyun-feature-for_esp8266_rtos_sdk_v2.x.x/components/aliyun/iotkit-embedded/src/packages
RULE_DIR := /home/ytzhitong/github/esp-aliyun-feature-for_esp8266_rtos_sdk_v2.x.x/components/aliyun/iotkit-embedded/build-rules
STRIP := xtensa-lx106-elf-strip
SYSROOT_INC := /home/ytzhitong/github/esp-aliyun-feature-for_esp8266_rtos_sdk_v2.x.x/components/aliyun/iotkit-embedded/.O/usr/include
TOP_DIR := /home/ytzhitong/github/esp-aliyun-feature-for_esp8266_rtos_sdk_v2.x.x/components/aliyun/iotkit-embedded

include $(RULE_DIR)/settings.mk
include $(CONFIG_TPL)

all:

LIBA_TARGET     := libiot_log.a
HDR_REFS        := src/sdk-impl

LIB_HEADERS     := $(wildcard */*.h)
PKG_SOURCE      := LITE-log.git
PKG_UPSTREAM    := git@gitlab.alibaba-inc.com:iot-middleware/LITE-log.git

env:
	@echo ""
	@printf -- "-----------------------------------------------------------------\n"
	@$(foreach var,$(SHOW_ENV_VARS),$(call Dump_Var,$(var)))
	@printf -- "-----------------------------------------------------------------\n"
	@echo ""

include $(RULE_DIR)/rules.mk
