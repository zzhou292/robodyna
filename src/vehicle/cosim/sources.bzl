"""Exact retained co-simulation implementation families; no numerical copies."""

COSIM_GROUPS = {
    "base": {
        "sources": [
            "src/chrono_vehicle/cosim/ChVehicleCosimBaseNode.cpp",
            "src/chrono_vehicle/cosim/ChVehicleCosimWheeledMBSNode.cpp",
            "src/chrono_vehicle/cosim/ChVehicleCosimTrackedMBSNode.cpp",
            "src/chrono_vehicle/cosim/ChVehicleCosimTireNode.cpp",
            "src/chrono_vehicle/cosim/ChVehicleCosimTerrainNode.cpp",
            "src/chrono_vehicle/cosim/ChVehicleCosimDBPRig.cpp"
        ],
        "headers": [
            "src/chrono_vehicle/cosim/ChVehicleCosimBaseNode.h",
            "src/chrono_vehicle/cosim/ChVehicleCosimWheeledMBSNode.h",
            "src/chrono_vehicle/cosim/ChVehicleCosimTrackedMBSNode.h",
            "src/chrono_vehicle/cosim/ChVehicleCosimTireNode.h",
            "src/chrono_vehicle/cosim/ChVehicleCosimTerrainNode.h",
            "src/chrono_vehicle/cosim/ChVehicleCosimDBPRig.h",
            "src/chrono_vehicle/cosim/ChVehicleCosimOtherNode.h"
        ]
    },
    "mbs": {
        "sources": [
            "src/chrono_vehicle/cosim/mbs/ChVehicleCosimRigNode.cpp",
            "src/chrono_vehicle/cosim/mbs/ChVehicleCosimWheeledVehicleNode.cpp",
            "src/chrono_vehicle/cosim/mbs/ChVehicleCosimTrackedVehicleNode.cpp"
        ],
        "headers": [
            "src/chrono_vehicle/cosim/mbs/ChVehicleCosimRigNode.h",
            "src/chrono_vehicle/cosim/mbs/ChVehicleCosimWheeledVehicleNode.h",
            "src/chrono_vehicle/cosim/mbs/ChVehicleCosimTrackedVehicleNode.h"
        ]
    },
    "robot": {
        "sources": [
            "src/chrono_vehicle/cosim/mbs/ChVehicleCosimViperNode.cpp",
            "src/chrono_vehicle/cosim/mbs/ChVehicleCosimCuriosityNode.cpp"
        ],
        "headers": [
            "src/chrono_vehicle/cosim/mbs/ChVehicleCosimViperNode.h",
            "src/chrono_vehicle/cosim/mbs/ChVehicleCosimCuriosityNode.h"
        ]
    },
    "rigid_tires": {
        "sources": [
            "src/chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeRigid.cpp",
            "src/chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeBypass.cpp"
        ],
        "headers": [
            "src/chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeRigid.h",
            "src/chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeBypass.h"
        ]
    },
    "flexible_tires": {
        "sources": [
            "src/chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeFlexible.cpp"
        ],
        "headers": [
            "src/chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeFlexible.h"
        ]
    },
    "cpu_terrain": {
        "sources": [
            "src/chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeChrono.cpp",
            "src/chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeRigid.cpp",
            "src/chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeSCM.cpp"
        ],
        "headers": [
            "src/chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeChrono.h",
            "src/chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeRigid.h",
            "src/chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeSCM.h"
        ]
    },
    "omp_terrain": {
        "sources": [
            "src/chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularOMP.cpp"
        ],
        "headers": [
            "src/chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularOMP.h"
        ]
    },
    "sph_terrain": {
        "sources": [
            "src/chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularSPH.cpp"
        ],
        "headers": [
            "src/chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularSPH.h"
        ]
    },
    "dem_terrain": {
        "sources": [
            "src/chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularDEM.cpp"
        ],
        "headers": [
            "src/chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularDEM.h"
        ]
    }
}
