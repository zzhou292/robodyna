cmake_minimum_required(VERSION 3.22)
project(RoboDynaSourceAssemblyFlight LANGUAGES CXX CUDA)
enable_testing()
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
get_filename_component(robo_flight_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include("${robo_flight_root}/case/source_assembly/SourceAssemblyBindings.cmake")
include("${robo_flight_root}/case/shell_collection/ShellCollectionContactGeometry.cmake")
if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_src/elements/ShellResidentLimits.h")
  message(FATAL_ERROR "Source GPU flight requires the explicitly selected TL active shell collection implementation")
endif()
add_subdirectory("${ROBO_DYNA_TL_ROOT}/lib_src/solvers" "tl-owner")
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/qeph/QephBatch.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/t3/T3Batch.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/ShellBatchPublication.cmake")
set(ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY "" CACHE FILEPATH "Frozen authenticated six-part Yaris inventory")
if(NOT EXISTS "${ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY}")
  message(FATAL_ERROR "Actual-source flight requires the explicit frozen inventory path")
endif()
add_executable(robo_dyna_source_assembly_flight_check
  SourceAssemblyFlightFixture.cu SourceAssemblyFlightChecks.cpp SourceAssemblyParentChecks.cpp
  SourceAssemblyFlightTest.cpp SourceAssemblyGroupFlightTest.cpp)
target_link_libraries(robo_dyna_source_assembly_flight_check PRIVATE
  robo_dyna_source_assembly_bindings robo_dyna_shell_collection_contact_geometry tl_shell_batch_publication
  CUDA::cudart GTest::gtest_main)
set_target_properties(robo_dyna_source_assembly_flight_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(robo_dyna_source_assembly_flight_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME source_assembly_flight COMMAND robo_dyna_source_assembly_flight_check)
set_tests_properties(source_assembly_flight PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180
  ENVIRONMENT "ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY=${ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY}")

set(ROBO_DYNA_SOURCE_ANALYTIC_INVENTORY "" CACHE FILEPATH "Frozen original analytic LAW44 part inventory")
set(ROBO_DYNA_SOURCE_ANALYTIC_MIXED_INVENTORY "" CACHE FILEPATH "Frozen original mixed table/analytic inventory")
if(ROBO_DYNA_SOURCE_ANALYTIC_INVENTORY OR ROBO_DYNA_SOURCE_ANALYTIC_MIXED_INVENTORY)
  foreach(fixture ROBO_DYNA_SOURCE_ANALYTIC_INVENTORY ROBO_DYNA_SOURCE_ANALYTIC_MIXED_INVENTORY)
    if(NOT EXISTS "${${fixture}}")
      message(FATAL_ERROR "Analytic source flight requires explicit ${fixture}")
    endif()
  endforeach()
  target_sources(robo_dyna_source_assembly_flight_check PRIVATE SourceAnalyticFlightTest.cpp)
  set_property(TEST source_assembly_flight APPEND PROPERTY ENVIRONMENT
    "ROBO_DYNA_SOURCE_ANALYTIC_INVENTORY=${ROBO_DYNA_SOURCE_ANALYTIC_INVENTORY}"
    "ROBO_DYNA_SOURCE_ANALYTIC_MIXED_INVENTORY=${ROBO_DYNA_SOURCE_ANALYTIC_MIXED_INVENTORY}")
endif()
