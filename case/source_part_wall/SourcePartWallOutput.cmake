include_guard(GLOBAL)
if(NOT TARGET robo_dyna_source_part_elastic_artifacts OR NOT TARGET robo_dyna_source_part_wall_contact)
  message(FATAL_ERROR "Wall output requires the existing source case/artifacts and wall contributor")
endif()
add_library(robo_dyna_source_part_wall_artifacts STATIC
  "${CMAKE_CURRENT_LIST_DIR}/../SourcePartWallArtifacts.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../SourcePartWallFields.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../SourcePartWallSetupFields.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../SourcePartWallContactFields.cpp")
target_link_libraries(robo_dyna_source_part_wall_artifacts PUBLIC
  robo_dyna_source_part_elastic_artifacts robo_dyna_source_part_wall_contact)
target_compile_options(robo_dyna_source_part_wall_artifacts PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo_dyna_source_part_wall "${CMAKE_CURRENT_LIST_DIR}/main.cpp")
target_link_libraries(robo_dyna_source_part_wall PRIVATE robo_dyna_source_part_wall_artifacts)
target_compile_options(robo_dyna_source_part_wall PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo_dyna_source_part_wall_output_check "${CMAKE_CURRENT_LIST_DIR}/source_part_wall_output_check.cpp")
target_link_libraries(robo_dyna_source_part_wall_output_check PRIVATE
  robo_dyna_source_part_wall_artifacts robo_dyna_accepted_replay GTest::gtest)
target_compile_options(robo_dyna_source_part_wall_output_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME source_part_wall_output COMMAND robo_dyna_source_part_wall_output_check
  "${ROBO_DYNA_SOURCE_PART_READINESS}" "${CRASH_CANONICAL_WALL}")
set_tests_properties(source_part_wall_output PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE PROCESSORS 1
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
