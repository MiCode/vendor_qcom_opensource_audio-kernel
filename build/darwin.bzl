load(":audio_modules.bzl", "audio_modules")
load(":module_mgr.bzl", "define_target_modules")
load(":build/sun.bzl", "arch_modules", "arch_config_options")

device_modules = []
device_config_options = [
            "CONFIG_TARGET_PRODUCT_DARWIN",
]

def define_darwin():
    define_target_modules(
        target = "darwin",
        variants = ["consolidate", "perf"],
        registry = audio_modules,
        modules = arch_modules + device_modules,
        config_options = arch_config_options + device_config_options,
    )
