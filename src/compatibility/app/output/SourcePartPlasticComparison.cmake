include_guard(GLOBAL)
if(NOT TARGET robo_dyna_source_part_wall_comparison)
  message(FATAL_ERROR "Plastic sensitivity requires the shared accepted wall comparison utilities")
endif()
add_library(robo_dyna_source_part_plastic_comparison STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourcePartPlasticComparison.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourcePartPlasticComparisonInput.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourcePartPlasticComparisonIntervals.cpp")
target_link_libraries(robo_dyna_source_part_plastic_comparison PUBLIC robo_dyna_source_part_wall_comparison)
target_compile_options(robo_dyna_source_part_plastic_comparison PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo_dyna_source_part_plastic_compare "${CMAKE_CURRENT_LIST_DIR}/../case/source_part_plastic/compare.cpp")
target_link_libraries(robo_dyna_source_part_plastic_compare PRIVATE robo_dyna_source_part_plastic_comparison)
if(ROBO_DYNA_ENABLE_REPLAY_CHECKS OR ROBO_DYNA_ENABLE_SOURCE_PART_ELASTIC)
  find_package(GTest REQUIRED)
  add_executable(robo_dyna_source_part_plastic_comparison_check
    "${CMAKE_CURRENT_LIST_DIR}/source_part_plastic_comparison_check.cpp")
  target_link_libraries(robo_dyna_source_part_plastic_comparison_check PRIVATE
    robo_dyna_source_part_plastic_comparison GTest::gtest_main)
  target_compile_options(robo_dyna_source_part_plastic_comparison_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME source_part_plastic_comparison COMMAND robo_dyna_source_part_plastic_comparison_check)
  set_tests_properties(source_part_plastic_comparison PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1)
endif()
