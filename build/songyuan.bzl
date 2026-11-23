load(":audio_modules.bzl", "audio_modules")
load(":module_mgr.bzl", "define_target_modules")
load(":build/canoe.bzl", "arch_modules", "arch_config_options")

device_modules = []
device_config_options = [
    "CONFIG_TARGET_PRODUCT_SONGYUAN",
]

def define_songyuan():
    define_target_modules(
        target = "canoe",
        device = "songyuan",
        variants = ["consolidate", "perf"],
        registry = audio_modules,
        modules = arch_modules + device_modules,
        config_options = arch_config_options + device_config_options,
    )
