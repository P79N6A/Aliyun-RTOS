MODULE_NAME := sample
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

DEPENDS             := src/platform
HDR_REFS            += src/sdk-impl src/utils src/log src/packages/LITE-utils
LDFLAGS             := -liot_sdk
LDFLAGS             += -liot_platform
ifneq (,$(filter -DIOTX_WITHOUT_ITLS,$(CFLAGS)))
LDFLAGS             += -Bstatic -liot_tls
endif 
CFLAGS              := $(filter-out -ansi,$(CFLAGS))

ifneq (,$(filter -D_PLATFORM_IS_WINDOWS_,$(CFLAGS)))
LDFLAGS             += -lws2_32
CFLAGS              := $(filter-out -DCOAP_COMM_ENABLED,$(CFLAGS))
endif

ifneq (,$(filter -DMQTT_COMM_ENABLED,$(CFLAGS)))
TARGET              += mqtt-example mqtt_rrpc-example mqtt_multi_thread-example
SRCS_mqtt-example   := mqtt/mqtt-example.c
SRCS_mqtt_rrpc-example := mqtt/mqtt_rrpc-example.c
SRCS_mqtt_multi_thread-example := mqtt/mqtt_multi_thread-example.c

    ifneq (,$(filter -DOTA_ENABLED,$(CFLAGS)))
    ifneq (,$(filter -DOTA_SIGNAL_CHANNEL=1,$(CFLAGS)))
    TARGET                += ota_mqtt-example
    SRCS_ota_mqtt-example := ota/ota_mqtt-example.c
    endif
    endif

    ifneq (,$(filter -DMQTT_SHADOW,$(CFLAGS)))
    TARGET              += shadow-example
    SRCS_shadow-example := device-shadow/shadow-example.c
    endif
    
    ifneq (,$(filter -DMQTT_ID2_AUTH,$(CFLAGS)))
    ifneq (,$(filter -DON_DAILY,$(CFLAGS)))
    LDFLAGS     += -ltfs
    else
    LDFLAGS     += -ltfs_online
    endif
    ifeq (,$(filter -DIOTX_WITHOUT_ITLS,$(CFLAGS)))
    LDFLAGS     += -litls
    endif
    ifeq (,$(filter -DIOTX_WITHOUT_TLS,$(CFLAGS)))
    LDFLAGS     += -liot_tls
    endif
    endif
    LDFLAGS     += -liot_sdk

endif

ifneq (,$(filter -DCOAP_COMM_ENABLED,$(CFLAGS)))
TARGET              += coap-example

SRCS_coap-example   := coap/coap-example.c
SRCS                += coap/coap-example.c

    ifneq (,$(filter -DOTA_ENABLED,$(CFLAGS)))
    ifneq (,$(filter -DOTA_SIGNAL_CHANNEL=2,$(CFLAGS)))
    TARGET                += ota_coap-example
    SRCS_ota_coap-example := ota/ota_coap-example.c
    endif
    endif
endif

ifneq (,$(filter -DHTTP_COMM_ENABLED,$(CFLAGS)))
TARGET              += http-example
SRCS_http-example   := http/http-example.c
endif

ifneq (,$(filter -DSUBDEVICE_ENABLED,$(CFLAGS)))
TARGET                += subdev-example
SRCS_subdev-example   += subdev/subdev-example.c \
                         subdev/subdev_example_api.c
endif

ifneq (,$(filter -DCLOUD_CONN_ENABLED,$(CFLAGS)))   
TARGET                    += cloud_conn-example 
SRCS_cloud_conn-example   := cloud-conn/cloud_conn-example.c
endif

ifneq (,$(filter -DCMP_ENABLED,$(CFLAGS)))   
TARGET                    += cmp-example 
SRCS_cmp-example          := cmp/cmp-example.c
endif


ifneq (,$(filter -DDM_ENABLED,$(CFLAGS)))   
TARGET                    += linkkit-example 
SRCS_linkkit-example      := linkkit/src/linkkit_export.c \
                             linkkit/src/lite_queue.c \
                             linkkit/samples/linkkit_sample.c
endif


env:
	@echo ""
	@printf -- "-----------------------------------------------------------------\n"
	@$(foreach var,$(SHOW_ENV_VARS),$(call Dump_Var,$(var)))
	@printf -- "-----------------------------------------------------------------\n"
	@echo ""

include $(RULE_DIR)/rules.mk
