include vendor/qcom/opensource/audio-kernel/audio_kernel_modules.mk
ifeq ($(ENABLE_AUDIO_LEGACY_TECHPACK),true)
include vendor/qcom/opensource/audio-kernel/legacy/audio_kernel_modules.mk
endif
BOARD_VENDOR_KERNEL_MODULES += $(AUDIO_KERNEL_MODULES)
