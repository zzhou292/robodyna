# External-project linkage follows chrono/template_project/CMakeLists.txt.
enable_language(CXX)
find_package(Chrono REQUIRED CONFIG)

if(CRASH_ENABLE_CHRONO_CHECK)
  add_executable(crash_chrono_wall_check wall_check.cpp)
  target_compile_features(crash_chrono_wall_check PRIVATE cxx_std_17)
  target_link_libraries(crash_chrono_wall_check PRIVATE ${CHRONO_TARGETS})

  set(CRASH_WALL_OBJ "" CACHE FILEPATH "Prepared pinned Yaris wall OBJ for the integration check")
  if(NOT EXISTS "${CRASH_WALL_OBJ}")
    message(FATAL_ERROR "CRASH_WALL_OBJ must name the compiled Yaris wall.obj")
  endif()
  add_test(NAME chrono_wall_integration COMMAND crash_chrono_wall_check "${CRASH_WALL_OBJ}")
  set_tests_properties(chrono_wall_integration PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
endif()

if(CRASH_ENABLE_CHRONO_REISSNER_CHECK)
  # FEA is a Chrono_core build option, not a find_package component. The test
  # checks CHRONO_FEA in the selected build's ChConfig.h and links its real
  # implementation. No copied formulation, dynamics, CUDA, or viewer target.
  find_package(GTest REQUIRED)
  add_executable(crash_chrono_reissner_reference_check reissner_reference_check.cpp)
  target_compile_features(crash_chrono_reissner_reference_check PRIVATE cxx_std_17)
  target_link_libraries(crash_chrono_reissner_reference_check PRIVATE Chrono::Chrono_core GTest::gtest_main)
  add_test(NAME chrono_reissner_reference COMMAND crash_chrono_reissner_reference_check)
  set_tests_properties(chrono_reissner_reference PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()

if(CRASH_ENABLE_CHRONO_SNAPSHOT_CHECK OR CRASH_ENABLE_TL_CHRONO_CHECK OR CRASH_ENABLE_TL_NODAL_CHECK OR CRASH_ENABLE_NORMAL_IMPACT)
  set(CRASH_TL_FEA_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Total-Lagrangian-FEA"
      CACHE PATH "TL-FEA source root providing the generic host nodal view")
  if(NOT EXISTS "${CRASH_TL_FEA_SOURCE_DIR}/lib_src/solvers/FENodalStateView.h")
    message(FATAL_ERROR "CRASH_TL_FEA_SOURCE_DIR must provide the real TL nodal header; no automatic fetch")
  endif()
  add_library(crash_accepted_surface STATIC AcceptedSurfaceMesh.cpp)
  target_compile_features(crash_accepted_surface PUBLIC cxx_std_17)
  target_include_directories(crash_accepted_surface PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${CRASH_TL_FEA_SOURCE_DIR}")
  target_link_libraries(crash_accepted_surface PUBLIC Chrono::Chrono_core)
  add_executable(crash_chrono_snapshot_check snapshot_check.cpp)
  target_link_libraries(crash_chrono_snapshot_check PRIVATE crash_accepted_surface)
  add_test(NAME chrono_snapshot_integration COMMAND crash_chrono_snapshot_check)
  set_tests_properties(chrono_snapshot_integration PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()

if(CRASH_ENABLE_TL_NODAL_CHECK OR CRASH_ENABLE_NORMAL_IMPACT)
  enable_language(CUDA)
  find_package(GTest REQUIRED)
  add_subdirectory("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/solvers" "tl-explicit-nodal-state")
  add_library(crash_nodal_mesh_output STATIC NodalMeshOutput.cpp)
  target_link_libraries(crash_nodal_mesh_output PUBLIC crash_accepted_surface tl_explicit_nodal_state)
  add_executable(crash_tl_nodal_output_check nodal_output_check.cu)
  set_target_properties(crash_tl_nodal_output_check PROPERTIES
    CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  # Chrono/Eigen mesh inspection is host-only; test kernels use TL POD views.
  target_compile_definitions(crash_tl_nodal_output_check PRIVATE EIGEN_NO_CUDA)
  target_link_libraries(crash_tl_nodal_output_check PRIVATE crash_nodal_mesh_output GTest::gtest_main)
  target_compile_options(crash_tl_nodal_output_check PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
  add_test(NAME tl_nodal_output_integration COMMAND crash_tl_nodal_output_check)
  set_tests_properties(tl_nodal_output_integration PROPERTIES
    TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1)
endif()

if(CRASH_ENABLE_TL_CHRONO_CHECK)
  # Explicitly opted-in test dependency, never part of the application adapter.
  # This composes targets from the TL source project without copying its code
  # or requiring an installed production backend that does not exist yet.
  set(TL_SHELL_ENABLE_PERSISTENT ON)
  add_subdirectory("${CRASH_TL_FEA_SOURCE_DIR}/lib_utest/qualification/shell"
                   "${CMAKE_CURRENT_BINARY_DIR}/tl-shell-qualification")
  find_package(GTest REQUIRED)
  add_executable(crash_tl_chrono_snapshot_check tl_snapshot_check.cpp)
  target_link_libraries(crash_tl_chrono_snapshot_check PRIVATE
    crash_accepted_surface persistent_shell GTest::gtest_main)
  add_test(NAME tl_chrono_snapshot_integration COMMAND crash_tl_chrono_snapshot_check)
  set_tests_properties(tl_chrono_snapshot_integration PROPERTIES
    TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1)
endif()
