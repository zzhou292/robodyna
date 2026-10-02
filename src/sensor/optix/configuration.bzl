"""Configure real OptiX/NVRTC and declare every runtime compiler header input."""

load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")

NVRTC_FLAGS = ["-use_fast_math", "-std=c++17", "-default-device", "-rdc", "true", "-D__x86_64"]

def _roots(headers, suffix):
    roots = {}
    for header in headers:
        if header.short_path.endswith("/" + suffix):
            roots[header.short_path[:-len(suffix) - 1]] = True
    if not roots:
        fail("Runtime compiler header closure lacks " + suffix)
    return sorted(roots.keys())

def _configuration_impl(ctx):
    runtime_headers = ctx.attr.cuda_runtime[CcInfo].compilation_context.headers
    cccl_headers = ctx.attr.cccl[CcInfo].compilation_context.headers
    optix_headers = ctx.attr.optix_sdk[CcInfo].compilation_context.headers
    # File.short_path is relative to the main repository runfiles root. External
    # headers naturally use ../<canonical-repository>/...; no SDK absolute path
    # or guessed Bzlmod repository name is embedded in the runtime configuration.
    directories = _roots(optix_headers.to_list(), "optix.h")
    directories += _roots(runtime_headers.to_list(), "cuda_runtime_api.h")
    directories += _roots(cccl_headers.to_list(), "thrust/version.h")
    directories += ["src/compatibility/chrono/src"]
    directories = list({directory: True for directory in directories}.keys())
    header = ctx.actions.declare_file("generated/chrono_sensor/ChConfigSensor.h")
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = header,
        substitutions = {
            "@CHRONO_HAS_OPTIX@": "#define CHRONO_HAS_OPTIX",
            "@CHRONO_HAS_VULKAN_RT@": "#undef CHRONO_HAS_VULKAN_RT",
            "@CHRONO_HAS_METAL_RT@": "#undef CHRONO_HAS_METAL_RT",
            "@CHRONO_HAS_SENSOR_RENDER@": "#define CHRONO_HAS_SENSOR_RENDER",
            "@VULKAN_SHADER_DIR@": "",
            "@SHADER_DIR@": "src/compatibility/chrono/src/chrono_sensor/optix/shaders",
            "@CUDA_NVRTC_INCLUDE_LIST@": ", ".join([json.encode(directory) for directory in directories] + ["0"]),
            "@CUDA_NVRTC_FLAG_LIST@": ", ".join([json.encode(flag) for flag in NVRTC_FLAGS] + ["0"]),
        },
    )
    metadata = ctx.actions.declare_file("nvrtc_inputs.json")
    ctx.actions.write(metadata, json.encode_indent({
        "schema": "robodyna.optix_nvrtc_inputs.v1",
        "cwd_contract": "main repository runfiles root",
        "include_directories": directories,
        "flags": NVRTC_FLAGS,
        "shader_directory": "src/compatibility/chrono/src/chrono_sensor/optix/shaders",
        "shaders": [source.short_path for source in ctx.files.shaders],
        "project_headers": [source.short_path for source in ctx.files.project_headers],
        "compiler_ir": "Retained production ChOptixUtils adds --optix-ir on CUDA12+; no driver/GPU call is needed for the separate compile coupon.",
    }, indent = "  ") + "\n")
    inputs = depset(ctx.files.shaders + ctx.files.project_headers + [metadata],
                    transitive = [runtime_headers, cccl_headers, optix_headers])
    return [
        DefaultInfo(files = depset([header]), runfiles = ctx.runfiles(transitive_files = inputs)),
        OutputGroupInfo(metadata = depset([metadata])),
    ]

optix_configuration = rule(
    implementation = _configuration_impl,
    attrs = {
        "template": attr.label(mandatory = True, allow_single_file = True),
        "shaders": attr.label_list(mandatory = True, allow_files = True),
        "project_headers": attr.label_list(mandatory = True, allow_files = True),
        "cuda_runtime": attr.label(default = Label("@rules_cuda//cuda:runtime"), providers = [CcInfo]),
        "cccl": attr.label(default = Label("@cuda_math//:cccl_headers"), providers = [CcInfo]),
        "optix_sdk": attr.label(default = Label("@optix_sdk//:headers"), providers = [CcInfo]),
    },
)
