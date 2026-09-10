# Application composition of existing TL owners. Native oracles are test-only.
include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/elements/qeph/QephBatch.cmake")
include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/elements/t3/T3Batch.cmake")
include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/elements/ShellBatchPublication.cmake")
add_library(robo_dyna_source_part_elastic_case STATIC
  SourcePartElasticCase.cpp SourcePartElasticInitialize.cpp SourcePartElasticLoading.cpp
  SourcePartElasticStep.cu SourcePartElasticObservation.cpp SourcePartElasticPilot.cpp)
target_link_libraries(robo_dyna_source_part_elastic_case PUBLIC
  robo_dyna_source_shell_collection tl_shell_batch_publication)
target_include_directories(robo_dyna_source_part_elastic_case PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../..")
set_target_properties(robo_dyna_source_part_elastic_case PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(robo_dyna_source_part_elastic_case PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")

add_library(robo_dyna_source_part_elastic_artifacts STATIC
  ../SourcePartElasticArtifacts.cpp ../SourcePartElasticFields.cpp)
target_link_libraries(robo_dyna_source_part_elastic_artifacts PUBLIC
  robo_dyna_source_part_elastic_case crash_nodal_mesh_output robo_dyna_artifact_io)
target_compile_options(robo_dyna_source_part_elastic_artifacts PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo_dyna_source_part_elastic main.cpp)
target_link_libraries(robo_dyna_source_part_elastic PRIVATE robo_dyna_source_part_elastic_artifacts)
find_package(GTest REQUIRED)
add_executable(robo_dyna_source_part_fields_check source_part_fields_check.cpp)
target_link_libraries(robo_dyna_source_part_fields_check PRIVATE
  robo_dyna_source_part_elastic_artifacts GTest::gtest_main)
add_test(NAME source_part_fields COMMAND robo_dyna_source_part_fields_check)
set_tests_properties(source_part_fields PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)

add_executable(robo_dyna_source_part_compare compare.cpp)
target_link_libraries(robo_dyna_source_part_compare PRIVATE robo_dyna_accepted_replay)
target_compile_options(robo_dyna_source_part_compare PRIVATE -fno-fast-math -ffp-contract=off)

if(TARGET qeph_q1_native AND TARGET t3_r3_native)
  enable_language(Fortran)
  find_package(GTest REQUIRED)
  add_executable(robo_dyna_source_part_elastic_check
    source_part_elastic_check.cu source_part_uniform_flight_check.cu SourcePartElasticNativeOracle.cpp)
  target_link_libraries(robo_dyna_source_part_elastic_check PRIVATE
    robo_dyna_source_part_elastic_artifacts qeph_q1_native t3_r3_native GTest::gtest)
  set_target_properties(robo_dyna_source_part_elastic_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(robo_dyna_source_part_elastic_check PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
  add_test(NAME source_part_elastic_native COMMAND robo_dyna_source_part_elastic_check "${ROBO_DYNA_SOURCE_PART_READINESS}")
  set_tests_properties(source_part_elastic_native PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
endif()
