#QTI_TECHPACK is used since kernel 5.15
ifeq ($(QTI_TECHPACK), true)
AUDIO_KERNEL_PATH=$(PWD)/../../../vendor/qcom/opensource/audio-kernel-ar/
AUDIO_ROOT=$(AUDIO_KERNEL_PATH)
# Use USERINCLUDE when you must reference the UAPI directories only.
USERINCLUDE     += \
                -I$(AUDIO_KERNEL_PATH)/include/uapi/audio

# Use LINUXINCLUDE when you must reference the include/ directory.
# Needed to be compatible with the O= option
INCS    += \
                -I$(AUDIO_KERNEL_PATH)/include/uapi \
                -I$(AUDIO_KERNEL_PATH)/include/uapi/audio \
                -I$(AUDIO_KERNEL_PATH)/include/asoc \
                -I$(AUDIO_KERNEL_PATH)/include

KBUILD_OPTIONS := AUDIO_ROOT=$(AUDIO_KERNEL_PATH)
else
CUR_MKFILE = $(abspath $(lastword $(MAKEFILE_LIST)))
AUDIO_ROOT=$(dir $(CUR_MKFILE))
UAPI_OUT=$(dir $(CUR_MKFILE))
HEADER_INSTALL_DIR=$(KERNEL_SRC)/scripts
KERNEL_BINARY_DIR=${KERNEL_UAPI_HEADERS_DIR}

KBUILD_OPTIONS := AUDIO_ROOT=$(dir $(CUR_MKFILE))
KBUILD_OPTIONS += UAPI_OUT=$(UAPI_OUT)
subdir-ccflags-y += -I$(AUDIO_ROOT)/include/uapi/
endif
KBUILD_OPTIONS += MODNAME=audio

AUDIO_KERNEL_HEADERS_PATH1 =  $(shell ls ./include/uapi/audio/linux/*.h)
AUDIO_KERNEL_HEADERS_PATH2 =  $(shell ls ./include/uapi/audio/linux/mfd/wcd9xxx/*.h)
AUDIO_KERNEL_HEADERS_PATH3 =  $(shell ls ./include/uapi/audio/sound/*.h)

ifeq ($(TARGET_SUPPORT),qcs40x)
KBUILD_OPTIONS += CONFIG_ARCH_QCS405=y
endif
ifeq ($(TARGET_SUPPORT), sdmsteppe)
KBUILD_OPTIONS += CONFIG_ARCH_SM6150=y
endif
ifeq ($(TARGET_SUPPORT), sdxlemur))
KBUILD_OPTIONS += CONFIG_ARCH_SDXLEMUR=y
endif
ifeq ($(TARGET_SUPPORT), sdxpinn))
KBUILD_OPTIONS += CONFIG_ARCH_SDXPINN=y
endif
ifeq ($(TARGET_SUPPORT), sa525m))
KBUILD_OPTIONS += CONFIG_ARCH_SA525M=y
endif
ifeq ($(TARGET_SUPPORT), qcm2290-mtp))
KBUILD_OPTIONS += CONFIG_SND_SOC_BENGAL=m
KBUILD_OPTIONS += CONFIG_ARCH_SCUBA=y
endif

obj-m := ipc/
obj-m += dsp/
obj-m += soc/
obj-m += asoc/
obj-m += asoc/codecs/
ifeq ($(TARGET_SUPPORT), $(filter $(TARGET_SUPPORT), sdmsteppe sdxlemur sdxpinn))
obj-m += asoc/codecs/wcd934x/
endif
ifeq ($(TARGET_SUPPORT), $(filter $(TARGET_SUPPORT), qcs40x))
obj-m += asoc/codecs/bolero/
obj-m += asoc/codecs/csra66x0/
obj-m += asoc/codecs/ep92/
endif
ifeq ($(TARGET_SUPPORT), sdmsteppe)
obj-m += asoc/codecs/bolero/
obj-m += asoc/codecs/wcd937x/
endif

ifeq ($(TARGET_SUPPORT), $(filter $(TARGET_SUPPORT), sa8155 sa8155ivi sa6155 sa8195 qtiquingvm))
KBUILD_OPTIONS += CONFIG_SND_SOC_AUTO=y
obj-m := ipc/
obj-m += dsp/
obj-m += asoc/
obj-m += asoc/codecs/
obj-m += soc/
ifeq ($(TARGET_SUPPORT), $(filter $(TARGET_SUPPORT), sa8155 sa8155ivi sa8195 qtiquingvm))
KBUILD_OPTIONS += CONFIG_SND_SOC_SA8155=m
endif
ifeq ($(TARGET_SUPPORT), $(filter $(TARGET_SUPPORT), sa6155))
KBUILD_OPTIONS += CONFIG_SND_SOC_SA6155=m
endif
endif

ifeq ($(AUTO_GVM), yes)
KBUILD_OPTIONS += CONFIG_SND_SOC_AUTO=y
KBUILD_OPTIONS += CONFIG_SND_SOC_GVM=y
endif

ifeq ($(TARGET_SUPPORT), $(filter $(TARGET_SUPPORT), lemans))
KBUILD_OPTIONS += CONFIG_SND_SOC_AUTO=y
KBUILD_OPTIONS += CONFIG_SND_SOC_SA8255=m
endif

define PROCESS_HEADERS
	$(foreach name,$(1),$(shell cd $(KERNEL_BINARY_DIR) && $(KERNEL_SRC)/scripts/headers_install.sh $(2)$(name) $(3)$(name)))
endef

all:
ifneq ($(QTI_TECHPACK), true)
	$(shell rm -fr $(shell pwd)/soc/core.h)
	$(shell ln -s $(KERNEL_SRC)/drivers/pinctrl/core.h $(shell pwd)/soc/core.h)
	$(shell rm -fr $(shell pwd)/include/soc/internal.h)
	$(shell ln -s $(KERNEL_SRC)/drivers/base/regmap/internal.h $(shell pwd)/include/soc/internal.h)
	$(shell rm -fr $(shell pwd)/soc/pinctrl-utils.h)
	$(shell ln -s $(KERNEL_SRC)/drivers/pinctrl/pinctrl-utils.h $(shell pwd)/soc/pinctrl-utils.h)
	$(shell rm -fr $(shell pwd)/include/soc/qcom/secure_buffer.h)
	$(shell ln -s $(KERNEL_SRC)/include/soc/qcom/secure_buffer.h $(shell pwd)/include/soc/qcom/secure_buffer.h)
	$(shell mkdir $(shell pwd)/linux)
	$(shell mkdir $(shell pwd)/sound)
	$(shell mkdir $(shell pwd)/linux/mfd)
	$(shell mkdir $(shell pwd)/linux/mfd/wcd9xxx)
	$(call PROCESS_HEADERS, $(notdir $(shell ls $(AUDIO_ROOT)/include/uapi/audio/linux/*.h)), $(AUDIO_ROOT)/include/uapi/audio/linux/, $(UAPI_OUT)/linux/)
	$(call PROCESS_HEADERS, $(notdir $(shell ls $(AUDIO_ROOT)/include/uapi/audio/linux/mfd/wcd9xxx/*.h)), $(AUDIO_ROOT)/include/uapi/audio/linux/mfd/wcd9xxx/, $(UAPI_OUT)/linux/mfd/wcd9xxx/)
	$(call PROCESS_HEADERS, $(notdir $(shell ls $(AUDIO_ROOT)/include/uapi/audio/sound/*.h)), $(AUDIO_ROOT)/include/uapi/audio/sound/, $(UAPI_OUT)/sound/)
	$(shell mkdir $(KERNEL_BINARY_DIR)/usr/include/audio)
	$(shell mkdir $(KERNEL_BINARY_DIR)/usr/include/audio/sound)
	$(shell mkdir $(KERNEL_BINARY_DIR)/usr/include/audio/linux)
	$(shell mkdir $(KERNEL_BINARY_DIR)/usr/include/audio/linux/mfd)
	$(shell mkdir $(KERNEL_BINARY_DIR)/usr/include/audio/linux/mfd/wcd9xxx)
	$(call PROCESS_HEADERS, $(notdir $(shell ls $(AUDIO_ROOT)/include/uapi/audio/linux/*.h)), $(AUDIO_ROOT)/include/uapi/audio/linux/, $(KERNEL_BINARY_DIR)/usr/include/audio/linux/)
	$(call PROCESS_HEADERS, $(notdir $(shell ls $(AUDIO_ROOT)/include/uapi/audio/linux/mfd/wcd9xxx/*.h)), $(AUDIO_ROOT)/include/uapi/audio/linux/mfd/wcd9xxx/, $(KERNEL_BINARY_DIR)/usr/include/audio/linux/mfd/wcd9xxx/)
	$(call PROCESS_HEADERS, $(notdir $(shell ls $(AUDIO_ROOT)/include/uapi/audio/sound/*.h)), $(AUDIO_ROOT)/include/uapi/audio/sound/, $(KERNEL_BINARY_DIR)/usr/include/audio/sound/)
endif
	$(MAKE) -C $(KERNEL_SRC) M=$(shell pwd) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(shell pwd) modules_install
ifeq ($(QTI_TECHPACK), true)
%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS)
endif

clean:
	@find . -type f \( -name "*.o" -o -name "*.ko" -o -name "*.mod" \
	-o -name "*.mod.c" -o -name "*.mod.o" -o -name ".*.cmd" \
	-o -name "Module.symvers" -o -name ".*.d" -o -name "modules.order" \) \
	-delete
	rm -rf .tmp_versions
