include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SourcePartWallContact.cmake")
add_executable(robo_dyna_source_part_wall_setup_check "${CMAKE_CURRENT_LIST_DIR}/source_part_wall_setup_check.cpp")
target_link_libraries(robo_dyna_source_part_wall_setup_check PRIVATE robo_dyna_source_part_wall_setup GTest::gtest)
add_executable(robo_dyna_source_part_wall_contact_check "${CMAKE_CURRENT_LIST_DIR}/source_part_wall_contact_check.cpp")
target_link_libraries(robo_dyna_source_part_wall_contact_check PRIVATE robo_dyna_source_part_wall_contact GTest::gtest)
foreach(check setup contact)
  target_compile_options(robo_dyna_source_part_wall_${check}_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME source_part_wall_${check} COMMAND robo_dyna_source_part_wall_${check}_check
    "${ROBO_DYNA_SOURCE_PART_READINESS}" "${CRASH_CANONICAL_WALL}")
  set_tests_properties(source_part_wall_${check} PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 90
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endforeach()
