"""Current locations of retained implementation units, independent of ownership.

Keys remain the original import identities used by the source inventories. Owning
targets and dependency boundaries are declared separately; moving a file does not
make its implementation a neutral service or an independent domain library.
"""

SOURCE_RELOCATIONS = {
    "src/chrono/physics/ChBodyAuxRef.cpp": "//src/mbd/bodies:RbBodyAuxRef.cpp",
    "src/chrono/physics/ChBodyEasy.cpp": "//src/mbd/bodies:RbBodyEasy.cpp",
    "src/chrono/physics/ChSystem.cpp": "//src/simulation/composition:RbSystem.cpp",
    "src/chrono/physics/ChSystemNSC.cpp": "//src/simulation/composition:RbSystemNSC.cpp",
    "src/chrono/physics/ChSystemSMC.cpp": "//src/simulation/composition:RbSystemSMC.cpp",

    "src/chrono/fea/ChMesh.cpp": "//src/fea/mesh:RbMesh.cpp",
    "src/chrono/physics/ChBody.cpp": "//src/mbd/bodies:RbBody.cpp",
    "src/chrono/physics/ChMassProperties.cpp": "//src/mechanics/inertia:RbMassProperties.cpp",
}

def current_source_label(original):
    return SOURCE_RELOCATIONS.get(original, "//src/compatibility/chrono:" + original)

def current_source_path(original):
    return current_source_label(original)[2:].replace(":", "/")
