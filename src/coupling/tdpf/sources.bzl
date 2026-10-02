"""Retained time-domain potential-flow FSI source and demo assets."""

TDPF_SOURCES = [
    "src/chrono_fsi/tdpf/ChFsiSystemTDPF.cpp",
    "src/chrono_fsi/tdpf/ChFsiInterfaceTDPF.cpp",
    "src/chrono_fsi/tdpf/ChFsiFluidSystemTDPF.cpp",
    "src/chrono_fsi/tdpf/impl/ChFsiFluidSystemTDPF_impl.cpp"
]

TDPF_HEADERS = [
    "src/chrono_fsi/tdpf/ChFsiSystemTDPF.h",
    "src/chrono_fsi/tdpf/ChFsiInterfaceTDPF.h",
    "src/chrono_fsi/tdpf/ChFsiFluidSystemTDPF.h",
    "src/chrono_fsi/tdpf/impl/ChFsiFluidSystemTDPF_impl.h"
]

TDPF_VISUAL = [
    "src/chrono_fsi/tdpf/visualization/ChTdpfVisualizationVSG.cpp",
    "src/chrono_fsi/tdpf/visualization/ChTdpfVisualizationVSG.h"
]

TDPF_ASSETS = [
    "data/fsi-tdpf/cparray/cparray.h5",
    "data/fsi-tdpf/oswec/base.obj",
    "data/fsi-tdpf/oswec/flap.obj",
    "data/fsi-tdpf/oswec/oswec.h5",
    "data/fsi-tdpf/rm3/float_cog.obj",
    "data/fsi-tdpf/rm3/plate_cog.obj",
    "data/fsi-tdpf/rm3/rm3.h5",
    "data/fsi-tdpf/sphere/eta.txt",
    "data/fsi-tdpf/sphere/sphere.h5",
    "data/fsi-tdpf/sphere/sphere.obj"
]

TDPF_EXPORTS = TDPF_SOURCES + TDPF_HEADERS + TDPF_VISUAL + TDPF_ASSETS + ["src/chrono_fsi/tdpf/ChFsiConfigTDPF.h.in", "src/chrono_fsi/tdpf/CMakeLists.txt"]
