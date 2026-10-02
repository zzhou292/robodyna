"""Source owner for the retained field-based FEA family.

This owner must share the aggregate host configuration. Its contact extension
changes existing class layouts and cannot be enabled by a demo-only define.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

FEA_MULTIPHYSICS_SOURCES = [
    "src/chrono/fea/multiphysics/ChNodeFEAfieldXYZ.cpp",
    "src/chrono/fea/multiphysics/ChFieldElement.cpp",
    "src/chrono/fea/multiphysics/ChFieldElementTetrahedron4.cpp",
    "src/chrono/fea/multiphysics/ChFieldElementTetrahedron4Face.cpp",
    "src/chrono/fea/multiphysics/ChFieldElementHexahedron8.cpp",
    "src/chrono/fea/multiphysics/ChFieldElementHexahedron8Face.cpp",
    "src/chrono/fea/multiphysics/ChFieldElementLoadableVolume.cpp",
    "src/chrono/fea/multiphysics/ChFieldElementLoadableSurface.cpp",
    "src/chrono/fea/multiphysics/ChFieldData.cpp",
    "src/chrono/fea/multiphysics/ChField.cpp",
    "src/chrono/fea/multiphysics/ChMaterial.cpp",
    "src/chrono/fea/multiphysics/ChMaterial3DDensity.cpp",
    "src/chrono/fea/multiphysics/ChMaterialPoisson.cpp",
    "src/chrono/fea/multiphysics/ChMaterial3DThermal.cpp",
    "src/chrono/fea/multiphysics/ChMaterial3DThermalLinear.cpp",
    "src/chrono/fea/multiphysics/ChMaterial3DThermalNonlinear.cpp",
    "src/chrono/fea/multiphysics/ChMaterial3DStress.cpp",
    "src/chrono/fea/multiphysics/ChMaterial3DHyperelastic.cpp",
    "src/chrono/fea/multiphysics/ChMaterial3DStressStVenant.cpp",
    "src/chrono/fea/multiphysics/ChMaterial3DStressNeoHookean.cpp",
    "src/chrono/fea/multiphysics/ChMaterial3DStressViscoNewton.cpp",
    "src/chrono/fea/multiphysics/ChMaterial3DStressViscoLinear.cpp",
    "src/chrono/fea/multiphysics/ChMaterial3DStressOgden.cpp",
    "src/chrono/fea/multiphysics/ChMaterial3DThermalStress.cpp",
    "src/chrono/fea/multiphysics/ChMaterial3DStressParallel.cpp",
    "src/chrono/fea/multiphysics/ChFEModel.cpp",
    "src/chrono/fea/multiphysics/ChFEModelThermal.cpp",
    "src/chrono/fea/multiphysics/ChFEModelDeformation.cpp",
    "src/chrono/fea/multiphysics/ChFEModelThermoDeformation.cpp",
    "src/chrono/fea/multiphysics/ChVisualDataExtractor.cpp",
    "src/chrono/fea/multiphysics/ChDrawer.cpp",
    "src/chrono/fea/multiphysics/ChLoaderHeatFlux.cpp",
    "src/chrono/fea/multiphysics/ChLoaderHeatRadiation.cpp",
    "src/chrono/fea/multiphysics/ChLoaderHeatConvection.cpp",
    "src/chrono/fea/multiphysics/ChLoaderHeatVolumetricSource.cpp",
    "src/chrono/fea/multiphysics/ChBuilderVolume.cpp",
    "src/chrono/fea/multiphysics/ChLinkFieldNode.cpp",
    "src/chrono/fea/multiphysics/ChSurfaceOfModel.cpp",
]

FEA_MULTIPHYSICS_HEADERS = [
    "src/chrono/fea/multiphysics/ChNodeFEAfieldXYZ.h",
    "src/chrono/fea/multiphysics/ChFieldElement.h",
    "src/chrono/fea/multiphysics/ChFieldElementTetrahedron4.h",
    "src/chrono/fea/multiphysics/ChFieldElementTetrahedron4Face.h",
    "src/chrono/fea/multiphysics/ChFieldElementHexahedron8.h",
    "src/chrono/fea/multiphysics/ChFieldElementHexahedron8Face.h",
    "src/chrono/fea/multiphysics/ChFieldElementLoadableVolume.h",
    "src/chrono/fea/multiphysics/ChFieldElementLoadableSurface.h",
    "src/chrono/fea/multiphysics/ChFieldData.h",
    "src/chrono/fea/multiphysics/ChField.h",
    "src/chrono/fea/multiphysics/ChMaterial.h",
    "src/chrono/fea/multiphysics/ChMaterial3DDensity.h",
    "src/chrono/fea/multiphysics/ChMaterialPoisson.h",
    "src/chrono/fea/multiphysics/ChMaterial3DThermal.h",
    "src/chrono/fea/multiphysics/ChMaterial3DThermalLinear.h",
    "src/chrono/fea/multiphysics/ChMaterial3DThermalNonlinear.h",
    "src/chrono/fea/multiphysics/ChMaterial3DStress.h",
    "src/chrono/fea/multiphysics/ChMaterial3DHyperelastic.h",
    "src/chrono/fea/multiphysics/ChMaterial3DStressStVenant.h",
    "src/chrono/fea/multiphysics/ChMaterial3DStressViscoNewton.h",
    "src/chrono/fea/multiphysics/ChMaterial3DStressViscoLinear.h",
    "src/chrono/fea/multiphysics/ChMaterial3DStressNeoHookean.h",
    "src/chrono/fea/multiphysics/ChMaterial3DStressOgden.h",
    "src/chrono/fea/multiphysics/ChMaterial3DThermalStress.h",
    "src/chrono/fea/multiphysics/ChMaterial3DStressParallel.h",
    "src/chrono/fea/multiphysics/ChFEModel.h",
    "src/chrono/fea/multiphysics/ChFEModelThermal.h",
    "src/chrono/fea/multiphysics/ChFEModelDeformation.h",
    "src/chrono/fea/multiphysics/ChFEModelThermoDeformation.h",
    "src/chrono/fea/multiphysics/ChVisualDataExtractor.h",
    "src/chrono/fea/multiphysics/ChDrawer.h",
    "src/chrono/fea/multiphysics/ChLoaderHeatFlux.h",
    "src/chrono/fea/multiphysics/ChLoaderHeatRadiation.h",
    "src/chrono/fea/multiphysics/ChLoaderHeatConvection.h",
    "src/chrono/fea/multiphysics/ChLoaderHeatVolumetricSource.h",
    "src/chrono/fea/multiphysics/ChBuilderVolume.h",
    "src/chrono/fea/multiphysics/ChLinkFieldNode.h",
    "src/chrono/fea/multiphysics/ChSurfaceOfModel.h",
]

def chrono_native_fea_multiphysics(name):
    """Compile each original field/material/model unit exactly once.

    The mixed native aggregate must depend on this owner after its shared
    ChConfig.h enables CHRONO_FEA_MULTIPHYSICS. Depend on headers here to
    avoid an implementation cycle with the existing contact-surface units.
    """
    cc_library(
        name = name,
        srcs = FEA_MULTIPHYSICS_SOURCES,
        hdrs = FEA_MULTIPHYSICS_HEADERS,
        includes = ["src"],
        deps = [":native_core_fea_headers"],
        alwayslink = True,
        local_defines = ["CH_API_COMPILE", "BT_THREADSAFE", "BP_USE_FIXEDPOINT_INT_32"],
        copts = ["-O3", "-fPIC", "-include", "$(location src/chrono/ChCorePCH.h)"],
        additional_compiler_inputs = ["src/chrono/ChCorePCH.h"],
        target_compatible_with = ["@platforms//os:linux"] + select({
            "//build_defs/features:fea_multiphysics_enabled": [],
            "//conditions:default": ["@platforms//:incompatible"],
        }),
        visibility = ["//visibility:public"],
        tags = ["manual", "native-fea-multiphysics", "cpu-only"],
    )
