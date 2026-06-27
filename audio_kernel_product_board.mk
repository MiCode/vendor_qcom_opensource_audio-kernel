# Build audio kernel driver

ifeq ($(call is-board-platform-in-list,kalama), true)
PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/q6_notifier_dlkm.ko\
	$(KERNEL_MODULES_OUT)/spf_core_dlkm.ko \
	$(KERNEL_MODULES_OUT)/audpkt_ion_dlkm.ko \
	$(KERNEL_MODULES_OUT)/gpr_dlkm.ko \
	$(KERNEL_MODULES_OUT)/msm_pcm_pcie_dlkm.ko \
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
	$(KERNEL_MODULES_OUT)/machine_dlkm.ko \
	$(KERNEL_MODULES_OUT)/swr_dmic_dlkm.ko \
	$(KERNEL_MODULES_OUT)/swr_haptics_dlkm.ko \
	$(KERNEL_MODULES_OUT)/hdmi_dlkm.ko \
	$(KERNEL_MODULES_OUT)/lpass_cdc_wsa2_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/lpass_cdc_wsa_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/lpass_cdc_va_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/lpass_cdc_rx_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/lpass_cdc_tx_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/lpass_cdc_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wsa884x_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wsa883x_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd938x_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd938x_slave_dlkm.ko
endif	# kalama

ifeq ($(call is-board-platform-in-list,bengal), true)
PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/q6_notifier_dlkm.ko\
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
	$(KERNEL_MODULES_OUT)/machine_dlkm.ko \
	$(KERNEL_MODULES_OUT)/bolero_cdc_dlkm.ko \
	$(KERNEL_MODULES_OUT)/rouleur_dlkm.ko \
        $(KERNEL_MODULES_OUT)/rouleur_slave_dlkm.ko \
        $(KERNEL_MODULES_OUT)/pm2250_spmi_dlkm.ko \
	$(KERNEL_MODULES_OUT)/va_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/tx_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/rx_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wsa881x_analog_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd938x_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd938x_slave_dlkm.ko
endif	# bengal

ifeq ($(call is-board-platform-in-list,monaco), true)
PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/q6_notifier_dlkm.ko\
	$(KERNEL_MODULES_OUT)/spf_core_dlkm.ko \
	$(KERNEL_MODULES_OUT)/audpkt_ion_dlkm.ko \
	$(KERNEL_MODULES_OUT)/gpr_dlkm.ko \
	$(KERNEL_MODULES_OUT)/audio_pkt_dlkm.ko \
	$(KERNEL_MODULES_OUT)/adsp_loader_dlkm.ko \
	$(KERNEL_MODULES_OUT)/audio_prm_dlkm.ko \
	$(KERNEL_MODULES_OUT)/q6_pdr_dlkm.ko \
	$(KERNEL_MODULES_OUT)/pinctrl_lpi_dlkm.ko \
	$(KERNEL_MODULES_OUT)/swr_dlkm.ko \
	$(KERNEL_MODULES_OUT)/swr_ctrl_dlkm.ko \
	$(KERNEL_MODULES_OUT)/snd_event_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd_core_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wcd9xxx_dlkm.ko \
	$(KERNEL_MODULES_OUT)/stub_dlkm.ko \
	$(KERNEL_MODULES_OUT)/machine_dlkm.ko \
	$(KERNEL_MODULES_OUT)/bolero_cdc_dlkm.ko \
	$(KERNEL_MODULES_OUT)/va_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/tx_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/rx_macro_dlkm.ko \
	$(KERNEL_MODULES_OUT)/wsa883x_dlkm.ko \
	$(KERNEL_MODULES_OUT)/pmw5100-spmi_dlkm.ko \
	$(KERNEL_MODULES_OUT)/besbev_dlkm.ko \
	$(KERNEL_MODULES_OUT)/besbev-slave_dlkm.ko

ifeq ($(TARGET_SUPPORTS_WEAR_AON), true)
PRODUCT_PACKAGES += \
	$(KERNEL_MODULES_OUT)/cc_dlkm.ko \
	$(KERNEL_MODULES_OUT)/audio_cc_ipc_dlkm.ko
endif	#wear_aon
endif	#monaco
ifeq ($(call is-board-platform-in-list,msmnile gen4), true)
ifneq (,$(filter $(TARGET_BOARD_PLATFORM)$(TARGET_BOARD_SUFFIX), msmnile_gvmq))
PRODUCT_PACKAGES  += $(KERNEL_MODULES_OUT)/machine_dlkm.ko \
        $(KERNEL_MODULES_OUT)/stub_dlkm.ko
else
PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/q6_notifier_dlkm.ko\
        $(KERNEL_MODULES_OUT)/spf_core_dlkm.ko \
        $(KERNEL_MODULES_OUT)/audpkt_ion_dlkm.ko \
        $(KERNEL_MODULES_OUT)/gpr_dlkm.ko \
        $(KERNEL_MODULES_OUT)/audio_pkt_dlkm.ko \
        $(KERNEL_MODULES_OUT)/q6_dlkm.ko \
        $(KERNEL_MODULES_OUT)/adsp_loader_dlkm.ko \
        $(KERNEL_MODULES_OUT)/audio_prm_dlkm.ko \
        $(KERNEL_MODULES_OUT)/snd_event_dlkm.ko \
        $(KERNEL_MODULES_OUT)/stub_dlkm.ko \
        $(KERNEL_MODULES_OUT)/machine_dlkm.ko \
        $(KERNEL_MODULES_OUT)/q6_pdr_dlkm.ko \
        $(KERNEL_MODULES_OUT)/coupled_ssr_dlkm.ko
endif   #msmnile
endif
ifeq ($(call is-board-platform-in-list,gen4), true)
PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/q6_notifier_dlkm.ko\
        $(KERNEL_MODULES_OUT)/spf_core_dlkm.ko \
        $(KERNEL_MODULES_OUT)/audpkt_ion_dlkm.ko \
        $(KERNEL_MODULES_OUT)/gpr_dlkm.ko \
        $(KERNEL_MODULES_OUT)/audio_pkt_dlkm.ko \
        $(KERNEL_MODULES_OUT)/q6_dlkm.ko \
        $(KERNEL_MODULES_OUT)/adsp_loader_dlkm.ko \
        $(KERNEL_MODULES_OUT)/audio_prm_dlkm.ko \
        $(KERNEL_MODULES_OUT)/q6_pdr_dlkm.ko \
        $(KERNEL_MODULES_OUT)/snd_event_dlkm.ko \
        $(KERNEL_MODULES_OUT)/stub_dlkm.ko \
        $(KERNEL_MODULES_OUT)/machine_dlkm.ko \
        $(KERNEL_MODULES_OUT)/pinctrl_lpi_dlkm.ko \
        $(KERNEL_MODULES_OUT)/wcd9xxx_dlkm.ko
endif   #gen4
