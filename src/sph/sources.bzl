"""Exact retained FSI/SPH source groups; numerical source files remain unchanged."""

SPH_CUDA_SOURCES = ['src/chrono_fsi/sph/ChFsiProblemSPH.cpp',
 'src/chrono_fsi/sph/ChFsiSystemSPH.cpp',
 'src/chrono_fsi/sph/ChFsiInterfaceSPH.cpp',
 'src/chrono_fsi/sph/ChFsiFluidSystemSPH.cpp',
 'src/chrono_fsi/sph/ChFsiSplashsurfSPH.cpp',
 'src/chrono_fsi/sph/physics/SphBceManager.cu',
 'src/chrono_fsi/sph/physics/SphCollisionSystem.cu',
 'src/chrono_fsi/sph/physics/SphDataManager.cu',
 'src/chrono_fsi/sph/physics/SphFluidDynamics.cu',
 'src/chrono_fsi/sph/physics/SphForce.cu',
 'src/chrono_fsi/sph/physics/SphForceWCSPH.cu',
 'src/chrono_fsi/sph/physics/SphForceISPH.cu',
 'src/chrono_fsi/sph/physics/SphFsiInterface.cu',
 'src/chrono_fsi/sph/physics/SphGeneral.cu',
 'src/chrono_fsi/sph/physics/SphParticleRelocator.cu',
 'src/chrono_fsi/sph/utils/SphUtilsDevice.cu',
 'src/chrono_fsi/sph/utils/SphUtilsPrint.cu']

SPH_HEADERS = ['src/chrono_fsi/sph/ChFsiDataTypesSPH.h',
 'src/chrono_fsi/sph/ChFsiDefinitionsSPH.h',
 'src/chrono_fsi/sph/ChFsiParamsSPH.h',
 'src/chrono_fsi/sph/ChFsiProblemSPH.h',
 'src/chrono_fsi/sph/ChFsiSystemSPH.h',
 'src/chrono_fsi/sph/ChFsiInterfaceSPH.h',
 'src/chrono_fsi/sph/ChFsiFluidSystemSPH.h',
 'src/chrono_fsi/sph/ChFsiPrintUtilsSPH.h',
 'src/chrono_fsi/sph/ChFsiSplashsurfSPH.h',
 'src/chrono_fsi/sph/physics/SphBceManager.cuh',
 'src/chrono_fsi/sph/physics/SphCollisionSystem.cuh',
 'src/chrono_fsi/sph/physics/SphDataManager.cuh',
 'src/chrono_fsi/sph/physics/SphFluidDynamics.cuh',
 'src/chrono_fsi/sph/physics/SphForce.cuh',
 'src/chrono_fsi/sph/physics/SphForceWCSPH.cuh',
 'src/chrono_fsi/sph/physics/SphForceISPH.cuh',
 'src/chrono_fsi/sph/physics/SphFsiInterface.cuh',
 'src/chrono_fsi/sph/physics/SphGeneral.cuh',
 'src/chrono_fsi/sph/physics/SphMarkerType.cuh',
 'src/chrono_fsi/sph/physics/SphParticleRelocator.cuh',
 'src/chrono_fsi/sph/math/SphLinearSolver.h',
 'src/chrono_fsi/sph/math/SphCustomMath.cuh',
 'src/chrono_fsi/sph/math/SphExactLinearSolvers.cuh',
 'src/chrono_fsi/sph/utils/SphUtilsDevice.cuh',
 'src/chrono_fsi/sph/utils/SphUtilsTypeConvert.cuh',
 'src/chrono_fsi/sph/utils/SphUtilsPrint.cuh',
 'src/chrono_fsi/sph/utils/SphUtilsLogging.cuh']

SPH_VISUAL_SOURCES = ['src/chrono_fsi/sph/visualization/ChSphVisualizationVSG.cpp']

SPH_VISUAL_HEADERS = ['src/chrono_fsi/sph/visualization/ChSphVisualizationVSG.h']

SPH_CONFIG_TEMPLATE = 'src/chrono_fsi/sph/ChFsiConfigSPH.h.in'
