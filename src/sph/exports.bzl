"""Exact first-party source exports for the native FSI/SPH compilation profile."""

load("//src/coupling/fsi:sources.bzl", "FSI_HEADERS", "FSI_SOURCES")
load(":sources.bzl", "SPH_CONFIG_TEMPLATE", "SPH_CUDA_SOURCES", "SPH_HEADERS", "SPH_VISUAL_HEADERS", "SPH_VISUAL_SOURCES")
load("//examples/sph:sources.bzl", "SPH_DEMO_SOURCES", "TDPF_DEMO_SOURCES")

FSI_SPH_SOURCE_EXPORTS = FSI_SOURCES + FSI_HEADERS + SPH_CUDA_SOURCES + SPH_HEADERS + SPH_VISUAL_SOURCES + SPH_VISUAL_HEADERS + [
    SPH_CONFIG_TEMPLATE,
    "src/chrono_fsi/CMakeLists.txt",
    "src/chrono_fsi/sph/CMakeLists.txt",
    "src/chrono_thirdparty/stb/stb_image.cpp",
    "src/chrono_thirdparty/stb/stb_image_write.cpp",
    "src/demos/fsi/sph/CMakeLists.txt",
    "src/demos/fsi/tdpf/CMakeLists.txt",
] + SPH_DEMO_SOURCES + TDPF_DEMO_SOURCES
