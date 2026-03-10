# Build audio kernel driver
ifeq ($(TARGET_USES_QMAA),true)
ifeq ($(TARGET_USES_QMAA_OVERRIDE_AUDIO),true)
BUILD_AUDIO_MODULES := true
else
BUILD_AUDIO_MODULES := false
endif
else
BUILD_AUDIO_MODULES := true
endif

ifeq ($(BUILD_AUDIO_MODULES),true)
ifneq ($(TARGET_BOARD_AUTO),true)
ifneq (,$(call is-board-platform-in-list2,$(TARGET_BOARD_PLATFORM)))
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/q6_notifier_dlkm.ko\
	$(KERNEL_MODULES_OUT)/spf_core_dlkm.ko \
	$(KERNEL_MODULES_OUT)/audpkt_ion_dlkm.ko \
	$(KERNEL_MODULES_OUT)/gpr_dlkm.ko \
	$(KERNEL_MODULES_OUT)/audio_pkt_dlkm.ko \
	$(KERNEL_MODULES_OUT)/q6_dlkm.ko \
	$(KERNEL_MODULES_OUT)/adsp_loader_dlkm.ko \
	$(KERNEL_MODULES_OUT)/audio_prm_dlkm.ko \
	$(KERNEL_MODULES_OUT)/q6_pdr_dlkm.ko \
	$(KERNEL_MODULES_OUT)/pinctrl_lpi_dlkm.ko \
	$(KERNEL_MODULES_OUT)/swr_dlkm.ko \
	$(KERNEL_MODULES_OUT)/swr_ctrl_dlkm.ko \
	$(KERNEL_MODULES_OUT)/snd_event_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd_core_dlkm.ko \
	$(KERNEL_MODULES_OUT)/mbhc_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd9xxx_dlkm.ko \
	$(KERNEL_MODULES_OUT)/stub_dlkm.ko \
	$(KERNEL_MODULES_OUT)/machine_dlkm.ko
ifneq (,$(call is-board-platform-in-list2, volcano))
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/lpass_bt_swr_dlkm.ko
endif
ifeq (,$(call is-board-platform-in-list2,bengal holi blair))
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/swr_haptics_dlkm.ko \
	$(KERNEL_MODULES_OUT)/hdmi_dlkm.ko \
	$(KERNEL_MODULES_OUT)/lpass_cdc_wsa2_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/lpass_cdc_wsa_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/lpass_cdc_va_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/lpass_cdc_rx_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/lpass_cdc_tx_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/lpass_cdc_dlkm.ko
ifeq (,$(call is-board-platform-in-list2,bengal holi blair pitti))
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/wsa884x_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wsa883x_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd937x_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd937x_slave_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd938x_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd938x_slave_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd9378_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd9378_slave_dlkm.ko \
	$(KERNEL_MODULES_OUT)/swr_dmic_dlkm.ko
endif
ifeq (,$(call is-board-platform-in-list2,niobe pitti))
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/wcd939x_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd939x_slave_dlkm.ko
endif

ifneq (,$(call is-board-platform-in-list2, pitti))
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/wsa881x_analog_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd9378_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd9378_slave_dlkm.ko
endif

endif
ifneq (,$(call is-board-platform-in-list2,bengal holi blair))
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/bolero_cdc_dlkm.ko \
	$(KERNEL_MODULES_OUT)/va_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/tx_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/rx_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wsa881x_analog_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd937x_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd937x_slave_dlkm.ko \
	$(KERNEL_MODULES_OUT)/sipa_dlkm.ko \
	$(KERNEL_MODULES_OUT)/sipa_tuning_dlkm.ko \
	$(KERNEL_MODULES_OUT)/fs15xxx_drv_dlkm.ko \
	$(KERNEL_MODULES_OUT)/fs15xxx_amp_dlkm.ko \
	$(KERNEL_MODULES_OUT)/lct_audio_info_dlkm.ko
endif
ifneq (,$(call is-board-platform-in-list2, holi blair))
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/wcd938x_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd938x_slave_dlkm.ko
endif
endif
else
ifneq (,$(call is-board-platform-in-list2, gen4 msmnile))
ifneq (,$(filter $(TARGET_BOARD_PLATFORM)$(TARGET_BOARD_SUFFIX), gen4_gvm msmnile_gvmq))
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/machine_dlkm.ko\
	$(KERNEL_MODULES_OUT)/stub_dlkm.ko
endif   #msmnile
endif
endif
endif
