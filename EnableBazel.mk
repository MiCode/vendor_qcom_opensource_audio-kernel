ifeq ($(call is-board-platform-in-list,monaco),true)
LOCAL_MODULE_DDK_BUILD := true

LOCAL_MODULE_KO_DIRS := dsp/q6_notifier_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/spf_core_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/audpkt_ion_dlkm.ko
LOCAL_MODULE_KO_DIRS += ipc/gpr_dlkm.ko
LOCAL_MODULE_KO_DIRS += ipc/audio_cc_ipc_dlkm.ko
LOCAL_MODULE_KO_DIRS += ipc/audio_pkt_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/adsp_loader_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/audio_prm_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/q6_pdr_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/pinctrl_lpi_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/swr_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/swr_ctrl_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/snd_event_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd_core_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd9xxx_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/stub_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/machine_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/bolero/bolero_cdc_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/bolero/va_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/bolero/tx_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/bolero/rx_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wsa883x/wsa883x_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/besbev/pmw5100-spmi_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/besbev/besbev_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/besbev/besbev-slave_dlkm.ko

ifeq ($(TARGET_SUPPORTS_WEAR_AON), true)
LOCAL_MODULE_KO_DIRS += cc_dlkm.ko
LOCAL_MODULE_KO_DIRS += audio_cc_ipc_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/cc/cc_dlkm.ko
LOCAL_MODULE_KO_DIRS += ipc/audio_cc_ipc_dlkm.ko
endif   #wear_aon
endif   #monaco
