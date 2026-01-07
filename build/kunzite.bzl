load(":audio_modules.bzl", "audio_modules")
load(":module_mgr.bzl", "define_target_modules")
load(":build/parrot.bzl", "arch_modules", "arch_config_options")

device_modules = []
device_config_options = []

def define_kunzite():
    define_target_modules(
        target = "kunzite",
        variants = ["consolidate", "perf"],
        registry = audio_modules,
        modules = arch_modules + device_modules,
        config_options = arch_config_options + device_config_options,
    )
