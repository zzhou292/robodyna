# Qualification composition only; mechanics and native references live in TL-FEA.
set(CRASH_TL_FEA_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../Total-Lagrangian-FEA"
  CACHE PATH "Owning TL-FEA source tree")
enable_language(CXX)
enable_language(CUDA)
enable_language(Fortran)
find_package(CUDAToolkit REQUIRED)
find_package(GTest REQUIRED)

# Compose the same production owner/batches used by the case. This option can
# stand alone: it does not require Chrono output or source-part case flags.
if(NOT TARGET tl_explicit_nodal_state)
  add_subdirectory("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/solvers"
    "${CMAKE_CURRENT_BINARY_DIR}/tl-nodal-owner")
endif()
include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/elements/ShellBatchBinding.cmake")
if(NOT TARGET tl_qeph_batch)
  include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/elements/qeph/QephBatch.cmake")
endif()
if(NOT TARGET tl_t3_batch)
  include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/elements/t3/T3Batch.cmake")
endif()
if(NOT TARGET tl_shell_batch_publication)
  include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/elements/ShellBatchPublication.cmake")
endif()

# Native recipes remain owned and verified by TL. The resident fixture needs
# both force-history references; plastic stabilization additionally consumes
# QEPH's actual prepared Fortran modules. Reuse earlier complete targets.
if(NOT TARGET qeph_q1_native)
  set(QEPH_Q1_BUILD_TESTS OFF)
  add_subdirectory("${CRASH_TL_FEA_SOURCE_DIR}/lib_utest/qualification/native/qeph"
    "${CMAKE_CURRENT_BINARY_DIR}/native-qeph" EXCLUDE_FROM_ALL)
endif()
if(NOT TARGET t3_r3_native)
  if(TARGET t3_r1_native OR TARGET t3_r2_native)
    message(FATAL_ERROR
      "Plastic resident qualification requires the native T3 force-history stage. "
      "An earlier source option created only a partial T3 subtree; enable "
      "ROBO_DYNA_SOURCE_T3_FORCE or configure plastic checks without that partial source qualification.")
  endif()
  set(T3_R1_BUILD_TESTS OFF)
  set(T3_R2_BUILD ON)
  set(T3_R2_BUILD_TESTS OFF)
  set(T3_R3_BUILD ON)
  set(T3_R3_BUILD_TESTS OFF)
  add_subdirectory("${CRASH_TL_FEA_SOURCE_DIR}/lib_utest/qualification/native/t3"
    "${CMAKE_CURRENT_BINARY_DIR}/native-t3" EXCLUDE_FROM_ALL)
endif()

include("${CRASH_TL_FEA_SOURCE_DIR}/lib_utest/qualification/materials/tabulated_shell/PointPlasticity.cmake")
add_subdirectory("${CRASH_TL_FEA_SOURCE_DIR}/lib_utest/qualification/shell_layered_j2"
  "${CMAKE_CURRENT_BINARY_DIR}/layered_shell")
include("${CRASH_TL_FEA_SOURCE_DIR}/lib_utest/qualification/resident_plasticity/ResidentPlasticity.cmake")
include("${CRASH_TL_FEA_SOURCE_DIR}/lib_utest/qualification/shell_layered_j2/PlasticStabilization.cmake")
