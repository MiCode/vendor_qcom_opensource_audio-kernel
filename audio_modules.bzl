load(":module_mgr.bzl", "create_module_registry")

DSP_PATH = "dsp"
IPC_PATH = "ipc"
SOC_PATH = "soc"
ASOC_PATH = "asoc"
ASOC_CODECS_PATH = ASOC_PATH + "/codecs"
ASOC_CODECS_BOLERO_PATH = ASOC_CODECS_PATH + "/bolero"

audio_modules = create_module_registry([":audio_headers"])
# ------------------------------------ AUDIO MODULE DEFINITIONS ---------------------------------
# >>>> DSP MODULES <<<<
audio_modules.register(
    name = "q6_notifier_dlkm",
    path = DSP_PATH,
    config_option = "CONFIG_MSM_QDSP6_NOTIFIER",
    srcs = [
        "audio_notifier.c",
        "audio_ssr.c",
    ],
)
audio_modules.register(
    name = "spf_core_dlkm",
    path = DSP_PATH,
    config_option = "CONFIG_SPF_CORE",
    srcs = ["spf-core.c"],
    conditional_srcs = {
        "CONFIG_DIGITAL_CDC_RSC_MGR": [
            "digital-cdc-rsc-mgr.c",
        ],
    },
)
audio_modules.register(
    name = "audpkt_ion_dlkm",
    path = DSP_PATH,
    config_option = "CONFIG_AUDIO_PKT_ION",
    srcs = ["msm_audio_ion.c"],
)
audio_modules.register(
    name = "adsp_loader_dlkm",
    path = DSP_PATH,
    config_option = "CONFIG_MSM_ADSP_LOADER",
    srcs = ["adsp-loader.c"],
)
audio_modules.register(
    name = "audio_prm_dlkm",
    path = DSP_PATH,
    config_option = "CONFIG_AUDIO_PRM",
    srcs = ["audio_prm.c"],
)
audio_modules.register(
    name = "q6_pdr_dlkm",
    path = DSP_PATH,
    config_option = "CONFIG_MSM_QDSP6_PDR",
    srcs = ["audio_pdr.c"],
)
# >>>> IPC MODULES <<<<
audio_modules.register(
    name = "gpr_dlkm",
    path = IPC_PATH,
    config_option = "CONFIG_MSM_QDSP6_GPR_RPMSG",
    srcs = ["gpr-lite.c"],
)
audio_modules.register(
    name = "audio_pkt_dlkm",
    path = IPC_PATH,
    config_option = "CONFIG_AUDIO_PKT",
    srcs = ["audio-pkt.c"],
)
audio_modules.register(
    name = "audio_cc_ipc_dlkm",
    path = IPC_PATH,
    config_option = "CONFIG_AUDIO_CC_IPC",
    conditional_srcs = {
        "TARGET_SUPPORTS_WEAR_AON": {
            True: [
                "audio-cc-ipc.c",
            ],
        },
    conditional_srcs = {
        "CONFIG_AUDIO_CC_IPC":[
                "audio-cc-ipc.c",
            ],
    },
)
# >>>> SOC MODULES <<<<
audio_modules.register(
    name = "pinctrl_lpi_dlkm",
    path = SOC_PATH,
    config_option = "CONFIG_PINCTRL_LPI",
    srcs = ["pinctrl-lpi.c"],
)
audio_modules.register(
    name = "swr_dlkm",
    path = SOC_PATH,
    config_option = "CONFIG_SOUNDWIRE",
    srcs = [
        "regmap-swr.c",
        "soundwire.c",
    ],
)
audio_modules.register(
    name = "swr_ctrl_dlkm",
    path = SOC_PATH,
    conditional_srcs = {
        "CONFIG_SOUNDWIRE_WCD_CTRL": [
            "swr-wcd-ctrl.c",
        ],
        "CONFIG_SOUNDWIRE_MSTR_CTRL": [
            "swr-mstr-ctrl.c",
        ],
    },
)
audio_modules.register(
    name = "snd_event_dlkm",
    path = SOC_PATH,
    config_option = "CONFIG_SND_EVENT",
    srcs = ["snd_event.c"],
)
# >>>> ASOC/CODEC MODULES <<<<
audio_modules.register(
    name = "wcd_core_dlkm",
    path = ASOC_CODECS_PATH,
    conditional_srcs = {
        "CONFIG_WCD9XXX_CODEC_CORE": [
            "wcd9xxx-rst.c",
            "wcd9xxx-core-init.c",
            "wcd9xxx-core.c",
            "wcd9xxx-irq.c",
            "wcd9xxx-slimslave.c",
            "wcd9xxx-utils.c",
            "wcd9335-regmap.c",
            "wcd9335-tables.c",
            "msm-cdc-pinctrl.c",
            "msm-cdc-supply.c",
            "wcd934x/wcd934x-regmap.c",
            "wcd934x/wcd934x-tables.c",
        ],
        "CONFIG_WCD9XXX_CODEC_CORE_V2": [
            "wcd9xxx-core-init.c",
            "msm-cdc-pinctrl.c",
            "msm-cdc-supply.c",
        ],
        "CONFIG_SND_SOC_WCD_IRQ": [
            "wcd-irq.c",
        ],
    },
)
audio_modules.register(
    name = "wcd9xxx_dlkm",
    path = ASOC_CODECS_PATH,
    config_option = "CONFIG_SND_SOC_WCD9XXX_V2",
    srcs = [
        "wcdcal-hwdep.c",
        "wcd9xxx-soc-init.c",
        "audio-ext-clk-up.c",
    ],
    conditional_srcs = {
        "CONFIG_WCD9XXX_CODEC_CORE": {
            True: [
                "wcd9xxx-common-v2.c",
                "wcd9xxx-resmgr-v2.c",
                "wcd-dsp-utils.c",
                "wcd-dsp-mgr.c",
            ],
            False: [
                "wcd-clsh.c",
            ],
        },
    },
)
audio_modules.register(
    name = "stub_dlkm",
    path = ASOC_CODECS_PATH,
    config_option = "CONFIG_SND_SOC_MSM_STUB",
    srcs = ["msm_stub.c"],
)
# >>>> ASOC MODULES <<<<
audio_modules.register(
    name = "machine_dlkm",
    path = ASOC_PATH,
    srcs = ["msm_common.c"],
    conditional_srcs = {
        "CONFIG_SND_SOC_KALAMA": [
            "kalama.c",
        ],
        "CONFIG_SND_SOC_BENGAL": [
            "bengal.c",
        ],
        "CONFIG_SND_SOC_MONACO": [
            "monaco.c",
        ],
    },
)
# >>>> ASOC/CODECS/BOLERO MODULES <<<<
audio_modules.register(
    name = "bolero_cdc_dlkm",
    path = ASOC_CODECS_BOLERO_PATH,
    config_option = "CONFIG_SND_SOC_BOLERO",
    srcs = [
        "bolero-cdc.c",
        "bolero-cdc-utils.c",
        "bolero-cdc-regmap.c",
        "bolero-cdc-tables.c",
        "bolero-clk-rsc.c",
    ],
)
audio_modules.register(
    name = "va_macro_dlkm",
    path = ASOC_CODECS_BOLERO_PATH,
    config_option = "CONFIG_VA_MACRO",
    srcs = ["va-macro.c"],
)
audio_modules.register(
    name = "rx_macro_dlkm",
    path = ASOC_CODECS_BOLERO_PATH,
    config_option = "CONFIG_RX_MACRO",
    srcs = ["rx-macro.c"],
)
audio_modules.register(
    name = "tx_macro_dlkm",
    path = ASOC_CODECS_BOLERO_PATH,
    config_option = "CONFIG_TX_MACRO",
    srcs = ["tx-macro.c"],
)
# >>>> WSA883X MODULE <<<<
audio_modules.register(
    name = "wsa883x_dlkm",
    path = ASOC_CODECS_PATH + "/wsa883x",
    config_option = "CONFIG_SND_SOC_WSA883X",
    srcs = [
        "wsa883x.c",
        "wsa883x-regmap.c",
        "wsa883x-tables.c",
    ],
)
# >>>> BESBEV MODULE <<<<
audio_modules.register(
    name = "besbev_dlkm",
    path = ASOC_CODECS_PATH + "/besbev",
    config_option = "CONFIG_SND_SOC_BESBEV",
    srcs = [
        "besbev.c",
        "besbev-regmap.c",
        "besbev-tables.c",
    ],
)
audio_modules.register(
    name = "pmw5100-spmi_dlkm",
    path = ASOC_CODECS_PATH + "/besbev",
    config_option = "CONFIG_PMW5100_SPMI",
    srcs = ["pmw5100-spmi.c"],
)
audio_modules.register(
    name = "besbev-slave_dlkm",
    path = ASOC_CODECS_PATH + "/besbev",
    config_option = "CONFIG_SND_SOC_BESBEV_SLAVE",
    srcs = ["besbev-slave.c"],
)
# >>>> CC MODULE <<<<
audio_modules.register(
    name = "cc_dlkm",
    path = ASOC_CODECS_PATH + "/cc",
    config_option = "CONFIG_SND_SOC_CC",
    conditional_srcs = {
        "TARGET_SUPPORTS_WEAR_AON": {
            True: [
                "cc_codec.c",
                "cc_pktzr.c",
            ],
        },
   conditional_srcs = {
        "CONFIG_SND_SOC_CC": [
                "cc_codec.c",
                "cc_pktzr.c",
            ],
    },
)
