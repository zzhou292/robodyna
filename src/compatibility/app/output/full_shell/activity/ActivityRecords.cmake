include_guard(GLOBAL)
if(NOT TARGET robo_dyna_full_shell_records)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/.." "${CMAKE_CURRENT_BINARY_DIR}/activity_record_base")
endif()
add_library(robo_dyna_parent_activity STATIC "${CMAKE_CURRENT_LIST_DIR}/ActivityRecord.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ActivityFields.cpp" "${CMAKE_CURRENT_LIST_DIR}/ActivityIO.cpp" "${CMAKE_CURRENT_LIST_DIR}/ActivityPlan.cpp")
target_link_libraries(robo_dyna_parent_activity PUBLIC robo_dyna_full_shell_records)
target_compile_features(robo_dyna_parent_activity PUBLIC cxx_std_17)
target_compile_options(robo_dyna_parent_activity PRIVATE -fno-fast-math -ffp-contract=off)
