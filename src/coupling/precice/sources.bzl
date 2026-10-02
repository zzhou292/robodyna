"""Unchanged preCICE adapters, original programs and declared input assets."""

PRECICE_GROUPS = {
    "base": {
        "sources": [
            "src/chrono_precice/ChPreciceAdapter.cpp"
        ],
        "headers": [
            "src/chrono_precice/ChPreciceAdapter.h",
            "src/chrono_precice/ChApiPrecice.h"
        ]
    },
    "mbs": {
        "sources": [
            "src/chrono_precice/ChPreciceAdapterMbs.cpp"
        ],
        "headers": [
            "src/chrono_precice/ChPreciceAdapterMbs.h"
        ]
    },
    "sph": {
        "sources": [
            "src/chrono_precice/ChPreciceAdapterSph.cpp"
        ],
        "headers": [
            "src/chrono_precice/ChPreciceAdapterSph.h"
        ]
    }
}

PRECICE_ASSETS = [
    "data/precice/flap_openfoam/README.md",
    "data/precice/flap_openfoam/clean.sh",
    "data/precice/flap_openfoam/cleaning-tools.sh",
    "data/precice/flap_openfoam/fluid_openfoam/0/U",
    "data/precice/flap_openfoam/fluid_openfoam/0/p",
    "data/precice/flap_openfoam/fluid_openfoam/0/pointDisplacement",
    "data/precice/flap_openfoam/fluid_openfoam/LICENSE.txt",
    "data/precice/flap_openfoam/fluid_openfoam/clean.sh",
    "data/precice/flap_openfoam/fluid_openfoam/constant/dynamicMeshDict",
    "data/precice/flap_openfoam/fluid_openfoam/constant/transportProperties",
    "data/precice/flap_openfoam/fluid_openfoam/constant/turbulenceProperties",
    "data/precice/flap_openfoam/fluid_openfoam/run.sh",
    "data/precice/flap_openfoam/fluid_openfoam/system/blockMeshDict",
    "data/precice/flap_openfoam/fluid_openfoam/system/controlDict",
    "data/precice/flap_openfoam/fluid_openfoam/system/decomposeParDict",
    "data/precice/flap_openfoam/fluid_openfoam/system/fvSchemes",
    "data/precice/flap_openfoam/fluid_openfoam/system/fvSolution",
    "data/precice/flap_openfoam/fluid_openfoam/system/preciceDict",
    "data/precice/flap_openfoam/precice_config.xml",
    "data/precice/flap_openfoam/solid_chrono/clean.sh",
    "data/precice/flap_openfoam/solid_chrono/flap.txt",
    "data/precice/flap_openfoam/solid_chrono/mbs_model.yaml",
    "data/precice/flap_openfoam/solid_chrono/mbs_participant.yaml",
    "data/precice/flap_openfoam/solid_chrono/mbs_solver.yaml",
    "data/precice/flap_openfoam/solid_chrono/run.sh",
    "data/precice/sphere_drop/fluid_buoyancy/cfd_participant.yaml",
    "data/precice/sphere_drop/fluid_sph/sph_model.yaml",
    "data/precice/sphere_drop/fluid_sph/sph_participant.yaml",
    "data/precice/sphere_drop/fluid_sph/sph_solver.yaml",
    "data/precice/sphere_drop/precice_config_explicit.xml",
    "data/precice/sphere_drop/precice_config_implicit.xml",
    "data/precice/sphere_drop/solid_chrono/mbs_model.yaml",
    "data/precice/sphere_drop/solid_chrono/mbs_participant.yaml",
    "data/precice/sphere_drop/solid_chrono/mbs_solver.yaml",
    "data/testing/precice/participant_1.yaml",
    "data/testing/precice/participant_2.yaml",
    "data/testing/precice/test.xml"
]

PRECICE_EXPORTS = [
    "data/precice/flap_openfoam/README.md",
    "data/precice/flap_openfoam/clean.sh",
    "data/precice/flap_openfoam/cleaning-tools.sh",
    "data/precice/flap_openfoam/fluid_openfoam/0/U",
    "data/precice/flap_openfoam/fluid_openfoam/0/p",
    "data/precice/flap_openfoam/fluid_openfoam/0/pointDisplacement",
    "data/precice/flap_openfoam/fluid_openfoam/LICENSE.txt",
    "data/precice/flap_openfoam/fluid_openfoam/clean.sh",
    "data/precice/flap_openfoam/fluid_openfoam/constant/dynamicMeshDict",
    "data/precice/flap_openfoam/fluid_openfoam/constant/transportProperties",
    "data/precice/flap_openfoam/fluid_openfoam/constant/turbulenceProperties",
    "data/precice/flap_openfoam/fluid_openfoam/run.sh",
    "data/precice/flap_openfoam/fluid_openfoam/system/blockMeshDict",
    "data/precice/flap_openfoam/fluid_openfoam/system/controlDict",
    "data/precice/flap_openfoam/fluid_openfoam/system/decomposeParDict",
    "data/precice/flap_openfoam/fluid_openfoam/system/fvSchemes",
    "data/precice/flap_openfoam/fluid_openfoam/system/fvSolution",
    "data/precice/flap_openfoam/fluid_openfoam/system/preciceDict",
    "data/precice/flap_openfoam/precice_config.xml",
    "data/precice/flap_openfoam/solid_chrono/clean.sh",
    "data/precice/flap_openfoam/solid_chrono/flap.txt",
    "data/precice/flap_openfoam/solid_chrono/mbs_model.yaml",
    "data/precice/flap_openfoam/solid_chrono/mbs_participant.yaml",
    "data/precice/flap_openfoam/solid_chrono/mbs_solver.yaml",
    "data/precice/flap_openfoam/solid_chrono/run.sh",
    "data/precice/sphere_drop/fluid_buoyancy/cfd_participant.yaml",
    "data/precice/sphere_drop/fluid_sph/sph_model.yaml",
    "data/precice/sphere_drop/fluid_sph/sph_participant.yaml",
    "data/precice/sphere_drop/fluid_sph/sph_solver.yaml",
    "data/precice/sphere_drop/precice_config_explicit.xml",
    "data/precice/sphere_drop/precice_config_implicit.xml",
    "data/precice/sphere_drop/solid_chrono/mbs_model.yaml",
    "data/precice/sphere_drop/solid_chrono/mbs_participant.yaml",
    "data/precice/sphere_drop/solid_chrono/mbs_solver.yaml",
    "data/testing/precice/participant_1.yaml",
    "data/testing/precice/participant_2.yaml",
    "data/testing/precice/test.xml",
    "src/chrono_precice/CMakeLists.txt",
    "src/chrono_precice/ChApiPrecice.h",
    "src/chrono_precice/ChPreciceAdapter.cpp",
    "src/chrono_precice/ChPreciceAdapter.h",
    "src/chrono_precice/ChPreciceAdapterMbs.cpp",
    "src/chrono_precice/ChPreciceAdapterMbs.h",
    "src/chrono_precice/ChPreciceAdapterSph.cpp",
    "src/chrono_precice/ChPreciceAdapterSph.h",
    "src/chrono_precice/yaml_app/CMakeLists.txt",
    "src/chrono_precice/yaml_app/run_chrono_adapter.cpp",
    "src/demos/precice/CMakeLists.txt",
    "src/demos/precice/demo_PRECICE_flap_openfoam.cpp",
    "src/demos/precice/demo_PRECICE_sphere_drop.cpp"
]
