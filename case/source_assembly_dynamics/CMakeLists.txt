cmake_minimum_required(VERSION 3.22)
project(RoboDynaSourceAssemblyDynamics LANGUAGES CXX CUDA)
enable_testing()
find_package(Chrono REQUIRED CONFIG)
find_package(CUDAToolkit REQUIRED)
find_package(GTest REQUIRED)
get_filename_component(robo_dynamics_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include("${robo_dynamics_root}/case/source_assembly/SourceAssemblyBindings.cmake")
set(CRASH_TL_FEA_SOURCE_DIR "${ROBO_DYNA_TL_ROOT}")
add_subdirectory("${ROBO_DYNA_TL_ROOT}/lib_src/solvers" tl-owner)
include("${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyDynamics.cmake")
set(ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY "" CACHE FILEPATH "Frozen authenticated six-part source inventory")
set(ROBO_DYNA_SOURCE_ASSEMBLY_WALL "" CACHE FILEPATH "Frozen original finite wall manifest")
foreach(robo_required IN ITEMS ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY ROBO_DYNA_SOURCE_ASSEMBLY_WALL)
  if(NOT EXISTS "${${robo_required}}")
    message(FATAL_ERROR "Assembly dynamics qualification requires explicit ${robo_required}")
  endif()
endforeach()
include("${robo_dynamics_root}/case/CanonicalWallArtifacts.cmake")
add_executable(robo_dyna_source_assembly_dynamics_check
  tests/Fixture.cpp tests/WallCaseTest.cpp tests/WallRunTest.cpp tests/StepTimingTest.cpp
  "${robo_dynamics_root}/qualification/source_assembly/SourceAssemblyFlightFixture.cu")
target_link_libraries(robo_dyna_source_assembly_dynamics_check PRIVATE robo_dyna_source_assembly_dynamics
  robo_dyna_canonical_wall_artifacts GTest::gtest_main CUDA::cudart)
set_target_properties(robo_dyna_source_assembly_dynamics_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(robo_dyna_source_assembly_dynamics_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME source_assembly_dynamics COMMAND robo_dyna_source_assembly_dynamics_check)
set_tests_properties(source_assembly_dynamics PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 600
  ENVIRONMENT "ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY=${ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY};ROBO_DYNA_SOURCE_ASSEMBLY_WALL=${ROBO_DYNA_SOURCE_ASSEMBLY_WALL}")
