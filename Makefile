ifeq ($(TARGET_SUPPORT), lemans)
KBUILD_OPTIONS := CONFIG_SND_SOC_AUTO=y
KBUILD_OPTIONS += CONFIG_SND_SOC_SA8255=m
endif

ifeq ($(AUTO_GVM), yes)
KBUILD_OPTIONS += CONFIG_SND_SOC_AUTO=y
KBUILD_OPTIONS += CONFIG_SND_SOC_GVM=y
endif

ifeq ($(TARGET_SUPPORT), $(filter $(TARGET_SUPPORT), sdxpinn sdxpinn-cpe-wkk sdxbaagha sdxpinn-cpe-wkk-v1))

AUDIO_ROOT=$(KERNEL_SRC)/$(M)
UAPI_OUT=$(KERNEL_SRC)/$(M)
HEADER_INSTALL_DIR=$(KERNEL_SRC)/kernel_platform/msm-kernel/scripts
KERNEL_BINARY_DIR=$(KERNEL_SRC)/

KBUILD_OPTIONS := AUDIO_ROOT=$(AUDIO_ROOT)
KBUILD_OPTIONS += MODNAME=audio
KBUILD_OPTIONS += UAPI_OUT=$(UAPI_OUT)

AUDIO_KERNEL_HEADERS_PATH1 =  $(shell ls ./include/uapi/audio/linux/*.h)
AUDIO_KERNEL_HEADERS_PATH3 =  $(shell ls ./include/uapi/audio/sound/*.h)

ifeq ($(TARGET_SUPPORT), sdxpinn)
KBUILD_OPTIONS += CONFIG_ARCH_SDXPINN=y
endif

ifeq ($(TARGET_SUPPORT), sdxpinn-cpe-wkk)
KBUILD_OPTIONS += CONFIG_ARCH_SDXPINN=y
endif

ifeq ($(TARGET_SUPPORT), sdxpinn-cpe-wkk-v1)
KBUILD_OPTIONS += CONFIG_ARCH_SDXPINN=y
endif

ifeq ($(TARGET_SUPPORT), sdxbaagha)
KBUILD_OPTIONS += CONFIG_ARCH_SDXBAAGHA=y
endif

subdir-ccflags-y += -I$(AUDIO_ROOT)/include/uapi/
subdir-ccflags-y += -I$(AUDIO_ROOT)/include/uapi/

obj-m := ipc/
obj-m += dsp/
obj-m += soc/
obj-m += asoc/
obj-m += asoc/codecs/
obj-m += asoc/codecs/wcd934x/
obj-m += asoc/codecs/lpass-cdc/

define PROCESS_HEADERS
	$(foreach name,$(1),$(shell cd $(KERNEL_BINARY_DIR) && $(KERNEL_SRC)/kernel_platform/msm-kernel/scripts/headers_install.sh $(2)$(name) $(3)$(name)))
endef

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

else

M=$(PWD)
AUDIO_ROOT=$(KERNEL_SRC)/$(M)

KBUILD_OPTIONS+=  AUDIO_ROOT=$(AUDIO_ROOT)

all: clean modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@ $(KBUILD_OPTIONS)
endif
