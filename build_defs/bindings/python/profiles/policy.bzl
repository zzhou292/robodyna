"""Explicit shared ABI selection for new Python wrappers, never demo-only macros."""

# These binding profiles have not admitted multicore, extra FE fields or TDPF.
# Keep their different class/declaration closures rejected until tested together.
IMPLEMENTATION_PROFILE = select({
    "//build_defs/features:multicore_enabled": ["@platforms//:incompatible"],
    "//build_defs/features:fea_multiphysics_enabled": ["@platforms//:incompatible"],
    "//build_defs/features:fsi_tdpf_enabled": ["@platforms//:incompatible"],
    "//conditions:default": [],
})

# Passed identically to SWIG and generated C++. The actual implementation TUs
# receive these capabilities from their one owning generated configuration.
PROFILE_DEFINES = select({
    "//build_defs/features:yaml_enabled": ["CHRONO_HAS_YAML"], "//conditions:default": [],
}) + select({
    "//build_defs/features:vsg_enabled": ["CHRONO_VSG"], "//conditions:default": [],
}) + select({
    "//build_defs/features:fsi_sph_enabled": ["CHRONO_FSI", "CHRONO_FSI_SPH"], "//conditions:default": [],
})

PROFILE_IDENTITY = select({
    "//build_defs/features:yaml_enabled": ["yaml=1"], "//conditions:default": ["yaml=0"],
}) + select({
    "//build_defs/features:vsg_enabled": ["vsg=1"], "//conditions:default": ["vsg=0"],
}) + select({
    "//build_defs/features:fsi_sph_enabled": ["fsi_sph=1"], "//conditions:default": ["fsi_sph=0"],
}) + select({
    "//build_defs/features:opencrg_enabled": ["opencrg=1"], "//conditions:default": ["opencrg=0"],
}) + select({
    "//build_defs/features:sensor_optix": ["sensor=optix"], "//conditions:default": ["sensor=vulkan"],
})

ROS_DEFINES = ["CHRONO_ROS_HAS_VEHICLE", "CHRONO_ROS_HAS_ROBOT"] + select({
    "//build_defs/features:ros_sensor_enabled": ["CHRONO_SENSOR"], "//conditions:default": [],
}) + select({
    "//build_defs/features:ros_sensor_optix": ["CHRONO_HAS_OPTIX"], "//conditions:default": [],
}) + select({
    "//build_defs/features:ros_urdf_enabled": ["CHRONO_HAS_URDF"], "//conditions:default": [],
})
ROS_IDENTITY = select({
    "//build_defs/features:ros_sensor_enabled": ["ros_sensor=1"], "//conditions:default": ["ros_sensor=0"],
}) + select({
    "//build_defs/features:ros_urdf_enabled": ["ros_urdf=1"], "//conditions:default": ["ros_urdf=0"],
})
