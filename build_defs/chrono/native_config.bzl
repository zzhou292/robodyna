"""Generate inherited configuration headers for the explicit host-only profile."""

_DISABLED_FEATURES = [
    "CHRONO_FEA_MULTIPHYSICS", "CHRONO_CASCADE", "CHRONO_IRRLICHT", "CHRONO_VSG",
    "CHRONO_PARDISO_MKL", "CHRONO_MUMPS", "CHRONO_PARSERS", "CHRONO_MULTICORE",
    "CHRONO_POSTPROCESS", "CHRONO_PYTHON", "CHRONO_VEHICLE", "CHRONO_FSI",
    "CHRONO_FSI_SPH", "CHRONO_FSI_TDPF", "CHRONO_DEM", "CHRONO_SYNCHRONO",
    "CHRONO_SENSOR", "CHRONO_ROS", "CHRONO_MODAL", "CHRONO_FMI",
    "CHRONO_HAS_GNUPLOT", "CHRONO_OMP_FOUND", "CHRONO_OMP_VERSION", "CHRONO_OMP_20",
    "CHRONO_OMP_30", "CHRONO_OMP_40", "CHRONO_OPENMP_ENABLED", "CHRONO_HAS_SSE",
    "CHRONO_SSE_LEVEL", "CHRONO_SSE_1_0", "CHRONO_SSE_2_0", "CHRONO_SSE_3_0",
    "CHRONO_SSE_4_1", "CHRONO_SSE_4_2", "CHRONO_HAS_AVX", "CHRONO_AVX_LEVEL",
    "CHRONO_AVX_1_0", "CHRONO_AVX_2_0", "CHRONO_HAS_NEON", "CHRONO_HAS_FMA",
    "CHRONO_SIMD_ENABLED", "CHRONO_HAS_HDF5", "CHRONO_HAS_YAML", "CHRONO_HAS_CUDA",
    "CHRONO_CUDA_VERSION", "CHRONO_HAS_HIP", "CHRONO_HIP_VERSION", "CHRONO_HAS_THRUST",
    "CHRONO_THRUST_VERSION", "CHRONO_HAS_GTEST", "CHRONO_HAS_GBENCHMARK", "CHRONO_COLLISION",
]

def _configuration_impl(ctx):
    substitutions = {"@" + feature + "@": "#undef " + feature for feature in _DISABLED_FEATURES}
    substitutions["@CHRONO_FEA@"] = "#define CHRONO_FEA"
    if ctx.attr.fea_multiphysics:
        substitutions["@CHRONO_FEA_MULTIPHYSICS@"] = "#define CHRONO_FEA_MULTIPHYSICS"
    if ctx.attr.yaml:
        substitutions["@CHRONO_HAS_YAML@"] = "#define CHRONO_HAS_YAML"
        substitutions["@CHRONO_PARSERS@"] = "#define CHRONO_PARSERS"
    if ctx.attr.vsg:
        substitutions["@CHRONO_VSG@"] = "#define CHRONO_VSG"
    if ctx.attr.fsi_sph or ctx.attr.fsi_tdpf:
        substitutions["@CHRONO_FSI@"] = "#define CHRONO_FSI"
    if ctx.attr.fsi_tdpf:
        substitutions["@CHRONO_FSI_TDPF@"] = "#define CHRONO_FSI_TDPF"
    if ctx.attr.fsi_sph:
        substitutions["@CHRONO_FSI_SPH@"] = "#define CHRONO_FSI_SPH"
    if ctx.attr.multicore:
        for feature in ["CHRONO_MULTICORE", "CHRONO_COLLISION", "CHRONO_HAS_THRUST",
                        "CHRONO_OMP_FOUND", "CHRONO_OMP_20", "CHRONO_OMP_30", "CHRONO_OMP_40"]:
            substitutions["@" + feature + "@"] = "#define " + feature
        # The retained CMake capability probe reports levels through OpenMP4.0.
        substitutions["@CHRONO_OMP_VERSION@"] = '#define CHRONO_OMP_VERSION "4.0"'
        substitutions["@CHRONO_THRUST_VERSION@"] = '#define CHRONO_THRUST_VERSION "3.2.0"'
        substitutions["@CHRONO_OPENMP_ENABLED@"] = (
            "#define CHRONO_OPENMP_ENABLED\n" +
            "#if !defined(_OPENMP) || _OPENMP < 201307\n" +
            '#error "Use --config=multicore with an OpenMP4.0-capable host compiler"\n' +
            "#endif"
        )
    # Diagnostic paths from Bazel are already relative to its execution root.
    # An old absolute checkout-prefix length would corrupt __FILE__ reporting.
    substitutions["@SOURCE_PATH_SIZE@"] = "0"
    ctx.actions.expand_template(
        template = ctx.file.config_template,
        output = ctx.outputs.config_header,
        substitutions = substitutions,
    )
    ctx.actions.expand_template(
        template = ctx.file.version_template,
        output = ctx.outputs.version_header,
        substitutions = {
            "@CHRONO_VERSION@": "10.0.0",
            "@CHRONO_VERSION_MAJOR@": "10",
            "@CHRONO_VERSION_MINOR@": "0",
            "@CHRONO_VERSION_PATCH@": "0",
            "@CH_VERSION@": "0x00100000",
        },
    )
    return [DefaultInfo(files = depset([ctx.outputs.config_header, ctx.outputs.version_header]))]

native_configuration = rule(
    implementation = _configuration_impl,
    attrs = {
        "fea_multiphysics": attr.bool(default = False),
        "multicore": attr.bool(default = False),
        "fsi_sph": attr.bool(default = False),
        "fsi_tdpf": attr.bool(default = False),
        "yaml": attr.bool(default = False),
        "vsg": attr.bool(default = False),
        "config_template": attr.label(allow_single_file = True, mandatory = True),
        "version_template": attr.label(allow_single_file = True, mandatory = True),
        "config_header": attr.output(mandatory = True),
        "version_header": attr.output(mandatory = True),
    },
)
