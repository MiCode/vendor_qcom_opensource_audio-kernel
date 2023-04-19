load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")
load("//build/kernel/kleaf:kernel.bzl", "kernel_module",
                                        "kernel_modules_install",
                                        "ddk_module",
                                        "ddk_submodule")

def _create_module_conditional_src_map(conditional_srcs):
    processed_conditional_srcs = {}

    for conditional_src_name in conditional_srcs:
        conditional_src = conditional_srcs[conditional_src_name]

        if type(conditional_src) == "list":
            processed_conditional_srcs[conditional_src_name] = { True: conditional_src }
        else:
            processed_conditional_srcs[conditional_src_name] = conditional_src

    return processed_conditional_srcs

def _get_module_srcs(module, options, formatter):
    srcs = [] + module.srcs
    module_path = "{}/".format(module.path) if module.path else ""

    for option in module.conditional_srcs:
        is_option_enabled = option in options
        srcs.extend(module.conditional_srcs[option].get(is_option_enabled, []))

    return ["{}{}".format(module_path, formatter(src)) for src in srcs]

def _combine_target_module_options(modules, config_options):
    all_options = {option: True for option in config_options}
    all_options = all_options | {module.config_option: True for module in modules if module.config_option}

    return all_options

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

def define_target_modules(target, variant, registry, modules, config_options = []):
    formatter = lambda s : s.replace("%t", target)\
                            .replace("%v", variant)\
                            .replace("%b", "{}_{}".format(target, variant))
    kernel_build = "{}_{}".format(target, variant)
    kernel_build_label = "//msm-kernel:{}".format(kernel_build)
    modules = [registry.get(module_name) for module_name in modules]
    options = _combine_target_module_options(modules, config_options)
    headers = ["//msm-kernel:all_headers"] + registry.hdrs
    all_module_rules = []

    for module in modules:
        rule_name = "{}_{}".format(kernel_build, module.name)
        srcs = _get_module_srcs(module, options, formatter)
        deps = headers + [formatter(dep) for dep in module.deps]

        if not srcs:
            continue

        ddk_submodule(
            name = rule_name,
            srcs = srcs,
            out = "{}.ko".format(module.name),
            deps = deps,
            local_defines = options.keys(),
        )

        all_module_rules.append(rule_name)

    ddk_module(
        name = "{}_modules".format(kernel_build),
        kernel_build = kernel_build_label,
        deps = all_module_rules
    )

    copy_to_dist_dir(
        name = "{}_modules_dist".format(kernel_build),
        data = [":{}_modules".format(kernel_build)],
        dist_dir = "out/target/product/{}/dlkm/lib/modules/".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
        log = "info"
    )

def define_consolidate_gki_modules(target, registry, modules, config_options = []):
    define_target_modules(target, "consolidate", registry, modules, config_options)
    define_target_modules(target, "gki", registry, modules, config_options)