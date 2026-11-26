load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")
load("//build/kernel/kleaf:kernel.bzl", "ddk_module", "kernel_module_group")

def _create_module_conditional_src_map(conditional_srcs):
    processed_conditional_srcs = {}

    for conditional_src_name in conditional_srcs:
        conditional_src = conditional_srcs[conditional_src_name]

        if type(conditional_src) == "list":
            processed_conditional_srcs[conditional_src_name] = {True: conditional_src}
        else:
            processed_conditional_srcs[conditional_src_name] = conditional_src

    return processed_conditional_srcs

def _get_enabled_module_objs(registry, modules):
    undefined_modules = []
    enabled_module_objs = []

    for module_name in modules:
        module_obj = registry.get(module_name)

        if not module_obj:
            undefined_modules.append(module_name)
        else:
            enabled_module_objs.append(module_obj)

    if undefined_modules:
        fail("FAILED. Tried to enable the following undefined modules: \n{}".format("\n".join(undefined_modules)))
    else:
        return enabled_module_objs

def _get_module_srcs(module, options):
    srcs = [] + module.srcs
    module_path = "{}/".format(module.path) if module.path else ""

    for option in module.conditional_srcs:
        is_option_enabled = option in options
        srcs.extend(module.conditional_srcs[option].get(is_option_enabled, []))

    return ["{}{}".format(module_path, src) for src in srcs]

def _combine_target_module_options(enabled_modules, config_options):
    all_options = {option: True for option in config_options}
    modules_options = {module.config_option: True for module in enabled_modules if module.config_option}

    return all_options | modules_options

def _define_target_modules(target, device, variant, registry, modules, product = None, config_options = []):
    dep_formatter = lambda s: s.replace("%t", target) \
        .replace("%v", variant) \
        .replace("%p", product if product else "") \
        .replace("%b", "{}_{}".format(target, variant))
    rule_prefix = "{}_{}_{}".format(device, variant, product) if product else "{}_{}".format(device, variant)
    enabled_modules = _get_enabled_module_objs(registry, modules)
    options = _combine_target_module_options(enabled_modules, config_options)
    headers = select({
        "//build/qcom_build_extensions:qtisocrepo_true": [
            "//soc-repo:all_headers",
            "//soc-repo:{}_{}/drivers/firmware/qcom/qcom-scm".format(target, variant),
            "//soc-repo:{}_{}/drivers/pinctrl/qcom/pinctrl-msm".format(target, variant),
            "//soc-repo:{}_{}/drivers/soc/qcom/pdr_interface".format(target, variant),
            "//soc-repo:{}_{}/drivers/remoteproc/rproc_qcom_common".format(target, variant),
            "//soc-repo:{}_{}/drivers/base/regmap/qti-regmap-debugfs".format(target, variant),
            "//soc-repo:{}_{}/drivers/power/supply/qti_battery_charger".format(target, variant),
            "//soc-repo:{}_{}/drivers/soc/qcom/wcd_usbss_i2c".format(target, variant),
	    "//soc-repo:{}_{}/drivers/soc/qcom/fsa4480_i2c".format(target, variant),
            "//soc-repo:{}_{}/kernel/trace/qcom_ipc_logging".format(target, variant),
            "//soc-repo:{}_{}/drivers/soc/qcom/socinfo".format(target, variant),
            "//soc-repo:hwid_headers",
            "//soc-repo:{}_{}/drivers/misc/hwid".format(target, variant),
        ] + registry.hdrs,
        "//build/qcom_build_extensions:qtisocrepo_false": ["//msm-kernel:all_headers"] + registry.hdrs,
    })
    kernel_build = select({
        "//build/qcom_build_extensions:qtisocrepo_true": "//soc-repo:{}_{}_base_kernel".format(target, variant),
        "//build/qcom_build_extensions:qtisocrepo_false": "//msm-kernel:{}_{}".format(target, variant),
    })

    submodule_rules = []
    for module in enabled_modules:
        rule_name = "{}_{}".format(rule_prefix, module.name)
        srcs = _get_module_srcs(module, options)
        deps = headers + [dep_formatter(dep) for dep in module.deps]

        if not srcs:
            continue

        ddk_module(
            name = rule_name,
            kernel_build = kernel_build,
            srcs = srcs,
            out = "{}.ko".format(module.name),
            deps = deps,
            local_defines = options.keys(),
        )

        submodule_rules.append(rule_name)

    kernel_module_group(
        name = "{}_modules".format(rule_prefix),
        srcs = submodule_rules
    )

    copy_to_dist_dir(
        name = "{}_modules_dist".format(rule_prefix),
        data = submodule_rules,
        dist_dir = "out/target/product/{}/dlkm/lib/modules/".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
        log = "info",
    )

def create_module_registry(hdrs = []):
    module_map = {}

    def register(name, path = None, config_option = None, srcs = [], conditional_srcs = {}, deps = []):
        module_map[name] = struct(
            name = name,
            path = path,
            srcs = srcs,
            conditional_srcs = _create_module_conditional_src_map(conditional_srcs),
            config_option = config_option,
            deps = deps,
        )

    return struct(
        module_map = module_map,
        hdrs = hdrs,
        register = register,
        get = module_map.get,
    )

def define_target_modules(target, device, variants, registry, modules, config_options = [], products = []):
    copts = select({
        ":factory_build_v1": [
            "-DCONFIG_FACTORY_BUILD=1",
        ],
        "//conditions:default": [
        ],
    })
    for variant in variants:
        if products:
            for product in products:
                _define_target_modules(
                    target = target,
                    device = device,
                    variant = variant,
                    registry = registry,
                    modules = modules,
                    product = product,
                    config_options = config_options,
                )
        else:
            _define_target_modules(
                target = target,
                device = device,
                variant = variant,
                registry = registry,
                modules = modules,
                config_options = config_options,
            )
