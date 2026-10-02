"""Explicit source groups from retained core/Multicore CMake declarations."""

CORE_EXTENSION_SOURCES = [
    "src/chrono/collision/multicore/ChBroadphase.cpp",
    "src/chrono/collision/multicore/ChCollisionModelMulticore.cpp",
    "src/chrono/collision/multicore/ChCollisionSystemMulticore.cpp",
    "src/chrono/collision/multicore/ChCollisionUtilsBroadphase.cpp",
    "src/chrono/collision/multicore/ChCollisionUtilsMPR.cpp",
    "src/chrono/collision/multicore/ChCollisionUtilsPRIMS.cpp",
    "src/chrono/collision/multicore/ChNarrowphase.cpp",
    "src/chrono/collision/multicore/ChNarrowphaseMPR.cpp",
    "src/chrono/collision/multicore/ChNarrowphasePRIMS.cpp",
    "src/chrono/collision/multicore/ChRayTest.cpp",
    "src/chrono/multicore_math/matrix.cpp",
    "src/chrono/multicore_math/real2.cpp",
    "src/chrono/multicore_math/real3.cpp",
    "src/chrono/multicore_math/real4.cpp",
    "src/chrono/multicore_math/vec3.cpp"
]

CORE_EXTENSION_HEADERS = [
    "src/chrono/collision/multicore/ChBroadphase.h",
    "src/chrono/collision/multicore/ChCollisionData.h",
    "src/chrono/collision/multicore/ChCollisionModelMulticore.h",
    "src/chrono/collision/multicore/ChCollisionSystemMulticore.h",
    "src/chrono/collision/multicore/ChCollisionUtils.h",
    "src/chrono/collision/multicore/ChConvexShape.h",
    "src/chrono/collision/multicore/ChNarrowphase.h",
    "src/chrono/collision/multicore/ChRayTest.h",
    "src/chrono/multicore_math/matrix.h",
    "src/chrono/multicore_math/real.h",
    "src/chrono/multicore_math/real2.h",
    "src/chrono/multicore_math/real3.h",
    "src/chrono/multicore_math/real4.h",
    "src/chrono/multicore_math/simd.h",
    "src/chrono/multicore_math/simd_avx.h",
    "src/chrono/multicore_math/simd_non.h",
    "src/chrono/multicore_math/simd_sse.h",
    "src/chrono/multicore_math/thrust.h",
    "src/chrono/multicore_math/types.h",
    "src/chrono/multicore_math/utility.h"
]

MULTICORE_SOURCES = [
    "src/chrono_multicore/ChDataManager.cpp",
    "src/chrono_multicore/collision/ChCollisionSystemChronoMulticore.cpp",
    "src/chrono_multicore/collision/ChContactContainerMulticore.cpp",
    "src/chrono_multicore/collision/ChContactContainerMulticoreNSC.cpp",
    "src/chrono_multicore/collision/ChContactContainerMulticoreSMC.cpp",
    "src/chrono_multicore/constraints/ChConstraintBilateral.cpp",
    "src/chrono_multicore/constraints/ChConstraintRigidRigid.cpp",
    "src/chrono_multicore/constraints/ChConstraintUtils.cpp",
    "src/chrono_multicore/physics/Ch3DOFContainer.cpp",
    "src/chrono_multicore/physics/ChFluidContainer.cpp",
    "src/chrono_multicore/physics/ChParticleContainer.cpp",
    "src/chrono_multicore/physics/ChSystemMulticore.cpp",
    "src/chrono_multicore/physics/ChSystemMulticoreNSC.cpp",
    "src/chrono_multicore/physics/ChSystemMulticoreSMC.cpp",
    "src/chrono_multicore/solver/ChIterativeSolverMulticore.cpp",
    "src/chrono_multicore/solver/ChIterativeSolverMulticoreNSC.cpp",
    "src/chrono_multicore/solver/ChIterativeSolverMulticoreSMC.cpp",
    "src/chrono_multicore/solver/ChSchurProduct.cpp",
    "src/chrono_multicore/solver/ChSolverMulticore.cpp",
    "src/chrono_multicore/solver/ChSolverMulticoreAPGD.cpp",
    "src/chrono_multicore/solver/ChSolverMulticoreAPGDREF.cpp",
    "src/chrono_multicore/solver/ChSolverMulticoreBB.cpp",
    "src/chrono_multicore/solver/ChSolverMulticoreCG.cpp",
    "src/chrono_multicore/solver/ChSolverMulticoreGS.cpp",
    "src/chrono_multicore/solver/ChSolverMulticoreJacobi.cpp",
    "src/chrono_multicore/solver/ChSolverMulticoreMINRES.cpp",
    "src/chrono_multicore/solver/ChSolverMulticoreSPGQP.cpp"
]

MULTICORE_HEADERS = [
    "src/chrono_multicore/ChApiMulticore.h",
    "src/chrono_multicore/ChDataManager.h",
    "src/chrono_multicore/ChMeasures.h",
    "src/chrono_multicore/ChMulticoreDefines.h",
    "src/chrono_multicore/ChSettings.h",
    "src/chrono_multicore/ChTimerMulticore.h",
    "src/chrono_multicore/collision/ChCollisionSystemChronoMulticore.h",
    "src/chrono_multicore/collision/ChContactContainerMulticore.h",
    "src/chrono_multicore/collision/ChContactContainerMulticoreNSC.h",
    "src/chrono_multicore/collision/ChContactContainerMulticoreSMC.h",
    "src/chrono_multicore/constraints/ChConstraintBilateral.h",
    "src/chrono_multicore/constraints/ChConstraintRigidRigid.h",
    "src/chrono_multicore/constraints/ChConstraintUtils.h",
    "src/chrono_multicore/physics/Ch3DOFContainer.h",
    "src/chrono_multicore/physics/ChFluidKernels.h",
    "src/chrono_multicore/physics/ChSystemMulticore.h",
    "src/chrono_multicore/solver/ChIterativeSolverMulticore.h",
    "src/chrono_multicore/solver/ChSolverMulticore.h"
]

MULTICORE_DEMO_SOURCES = [
    "src/demos/multicore/demo_MCORE_ballsNSC.cpp",
    "src/demos/multicore/demo_MCORE_ballsSMC.cpp",
    "src/demos/multicore/demo_MCORE_callbackNSC.cpp",
    "src/demos/multicore/demo_MCORE_callbackSMC.cpp",
    "src/demos/multicore/demo_MCORE_collision_object.cpp",
    "src/demos/multicore/demo_MCORE_cratering.cpp",
    "src/demos/multicore/demo_MCORE_custom_collision.cpp",
    "src/demos/multicore/demo_MCORE_fluidNSC.cpp",
    "src/demos/multicore/demo_MCORE_friction.cpp",
    "src/demos/multicore/demo_MCORE_mesh_contact.cpp",
    "src/demos/multicore/demo_MCORE_mixerNSC.cpp",
    "src/demos/multicore/demo_MCORE_mixerSMC.cpp",
    "src/demos/multicore/demo_MCORE_motors.cpp",
    "src/demos/multicore/demo_MCORE_particlesNSC.cpp",
    "src/demos/multicore/demo_MCORE_snow.cpp"
]

MULTICORE_ASSETS = [
    "data/textures/blue.png",
    "data/textures/bluewhite.png",
    "data/textures/concrete.jpg",
    "data/textures/checker1.png",
    "data/vehicle/hmmwv/hmmwv_tire_fine.obj",
    "data/vehicle/hmmwv/hmmwv_tire_coarse.obj"
]

MULTICORE_EVIDENCE = [
    "src/chrono/CMakeLists.txt",
    "src/chrono_multicore/CMakeLists.txt",
    "src/demos/multicore/CMakeLists.txt",
    "src/chrono_multicore/ChConfigMulticore.h.in",
    "src/tests/unit_tests/multicore/utest_MCORE_gravity.cpp",
    "src/tests/unit_tests/multicore/utest_MCORE_real3.cpp",
    "src/tests/unit_tests/ut_utils.h"
]

MULTICORE_EXPORTS = CORE_EXTENSION_SOURCES + CORE_EXTENSION_HEADERS + MULTICORE_SOURCES + MULTICORE_HEADERS + MULTICORE_DEMO_SOURCES + MULTICORE_ASSETS + MULTICORE_EVIDENCE
