include_guard(GLOBAL)
target_sources(robo_dyna_source_part_elastic_case PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/SourcePartPlasticPilot.cpp")
target_sources(robo_dyna_source_part_wall_artifacts PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/../source_part_wall/SourcePartWallExecution.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../SourcePartPlasticFields.cpp")
add_executable(robo_dyna_source_part_plastic_wall "${CMAKE_CURRENT_LIST_DIR}/main.cpp")
target_link_libraries(robo_dyna_source_part_plastic_wall PRIVATE robo_dyna_source_part_wall_artifacts)
target_compile_options(robo_dyna_source_part_plastic_wall PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo_dyna_source_part_plastic_output_check
  "${CMAKE_CURRENT_LIST_DIR}/source_part_plastic_output_check.cpp")
target_link_libraries(robo_dyna_source_part_plastic_output_check PRIVATE
  robo_dyna_source_part_wall_artifacts robo_dyna_accepted_replay robo_dyna_source_part_material GTest::gtest)
target_compile_options(robo_dyna_source_part_plastic_output_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME source_part_plastic_output COMMAND robo_dyna_source_part_plastic_output_check
  "${ROBO_DYNA_SOURCE_PART_READINESS}" "${CRASH_CANONICAL_WALL}")
set_tests_properties(source_part_plastic_output PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
add_executable(robo_dyna_source_part_plastic_engine_check
  "${CMAKE_CURRENT_LIST_DIR}/source_part_plastic_engine_check.cpp")
target_link_libraries(robo_dyna_source_part_plastic_engine_check PRIVATE
  robo_dyna_source_part_elastic_case GTest::gtest)
target_compile_options(robo_dyna_source_part_plastic_engine_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME source_part_plastic_engine COMMAND robo_dyna_source_part_plastic_engine_check
  "${ROBO_DYNA_SOURCE_PART_READINESS}" "${CRASH_CANONICAL_WALL}")
set_tests_properties(source_part_plastic_engine PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
