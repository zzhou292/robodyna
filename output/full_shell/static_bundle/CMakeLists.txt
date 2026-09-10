add_library(robo_dyna_full_shell_source_mapping STATIC
  InputChecks.cpp CanonicalSpecs.cpp CanonicalSource.cpp CanonicalCatalog.cpp
  Scope.cpp ScopeParts.cpp CanonicalGeometry.cpp MappingArrays.cpp MappingDigest.cpp MappingTopology.cpp
  MappingEncoding.cpp PreparedSourceMapping.cpp MappingFields.cpp
  MappingRecordWriter.cpp MappingRecordReader.cpp)
target_link_libraries(robo_dyna_full_shell_source_mapping PUBLIC robo_dyna_full_shell_records)
target_compile_features(robo_dyna_full_shell_source_mapping PUBLIC cxx_std_17)
target_compile_options(robo_dyna_full_shell_source_mapping PRIVATE -fno-fast-math -ffp-contract=off)
option(ROBO_DYNA_SOURCE_MAPPING_TESTS "Build bounded source mapping host contracts" ${ROBO_DYNA_FULL_SHELL_RECORD_TESTS})
if(ROBO_DYNA_SOURCE_MAPPING_TESTS)
  find_package(GTest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(robo_dyna_source_mapping_check tests/MappingValueTest.cpp
    tests/ActualMappingTest.cpp tests/ActualMappingCorruptionTest.cpp)
  target_link_libraries(robo_dyna_source_mapping_check PRIVATE robo_dyna_full_shell_source_mapping GTest::gtest_main)
  target_compile_options(robo_dyna_source_mapping_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME full_shell_source_mapping_values COMMAND robo_dyna_source_mapping_check --gtest_filter=SourceMappingValues.*)
  set_tests_properties(full_shell_source_mapping_values PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1)
  set(ROBO_DYNA_SOURCE_MAPPING_CANONICAL_FIXTURE "" CACHE PATH "Explicit authenticated original Yaris canonical assets")
  set(ROBO_DYNA_SOURCE_MAPPING_SCOPE_FIXTURE "" CACHE FILEPATH "Explicit frozen no-tire source-scope report")
  if(ROBO_DYNA_SOURCE_MAPPING_CANONICAL_FIXTURE OR ROBO_DYNA_SOURCE_MAPPING_SCOPE_FIXTURE)
    if(NOT EXISTS "${ROBO_DYNA_SOURCE_MAPPING_CANONICAL_FIXTURE}/manifest.json" OR
       NOT EXISTS "${ROBO_DYNA_SOURCE_MAPPING_CANONICAL_FIXTURE}/source_model.zip" OR
       NOT EXISTS "${ROBO_DYNA_SOURCE_MAPPING_SCOPE_FIXTURE}")
      message(FATAL_ERROR "Complete explicit canonical/source-scope fixture paths are required")
    endif()
    add_test(NAME full_shell_source_mapping_actual COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_LIST_DIR}/tests/actual_source_fixture.py"
      "${ROBO_DYNA_SOURCE_MAPPING_CANONICAL_FIXTURE}" "${ROBO_DYNA_SOURCE_MAPPING_SCOPE_FIXTURE}"
      "$<TARGET_FILE:robo_dyna_source_mapping_check>")
    set_tests_properties(full_shell_source_mapping_actual PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE PROCESSORS 1)
  endif()
endif()
