cmake_minimum_required(VERSION 3.22)
project(RoboDynaImpactResponseAnalysis LANGUAGES CXX)

find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
enable_testing()

set(ROBO_DYNA_TL_ROOT "" CACHE PATH
    "Qualified TL-FEA source root for bounded host layout utilities")
if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_utils/BoundedArena.h")
  message(FATAL_ERROR "Impact response analysis requires an explicit qualified TL-FEA root")
endif()

get_filename_component(ROBO_DYNA_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(ROBO_DYNA_FULL_SHELL_RECORD_TESTS OFF CACHE BOOL
    "Do not duplicate owning record tests in the focused analysis cache" FORCE)
set(ROBO_DYNA_SOURCE_MAPPING_TESTS OFF CACHE BOOL
    "Do not duplicate owning source mapping tests in the focused analysis cache" FORCE)
add_subdirectory("${ROBO_DYNA_ROOT}/output/physical_run"
                 "${CMAKE_CURRENT_BINARY_DIR}/physical-run")

add_library(robo_dyna_impact_response_analysis STATIC
  ParentCatalog.cpp Plasticity.cpp Connectivity.cpp RunMetadata.cpp
  RunSummary.cpp Report.cpp RunAnalysis.cpp)
target_link_libraries(robo_dyna_impact_response_analysis
  PUBLIC robo_dyna_physical_run_records)
target_include_directories(robo_dyna_impact_response_analysis
  PRIVATE "${ROBO_DYNA_TL_ROOT}")
target_compile_features(robo_dyna_impact_response_analysis PUBLIC cxx_std_17)
target_compile_options(robo_dyna_impact_response_analysis
  PRIVATE -fno-fast-math -ffp-contract=off)

add_executable(robo_dyna_impact_response_analysis_cli main.cpp)
set_target_properties(robo_dyna_impact_response_analysis_cli PROPERTIES
  OUTPUT_NAME robo_dyna_impact_response_analysis)
target_link_libraries(robo_dyna_impact_response_analysis_cli
  PRIVATE robo_dyna_impact_response_analysis)
target_compile_options(robo_dyna_impact_response_analysis_cli
  PRIVATE -fno-fast-math -ffp-contract=off)

add_executable(robo_dyna_impact_response_analysis_check
  tests/AnalysisTest.cpp)
target_link_libraries(robo_dyna_impact_response_analysis_check
  PRIVATE robo_dyna_impact_response_analysis GTest::gtest_main)
target_compile_options(robo_dyna_impact_response_analysis_check
  PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME impact_response_analysis
  COMMAND robo_dyna_impact_response_analysis_check)
set_tests_properties(impact_response_analysis PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
