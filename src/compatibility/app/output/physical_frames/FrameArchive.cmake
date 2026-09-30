include_guard(GLOBAL)
if(NOT TARGET robo_dyna_full_shell_source_bundle)
  set(ROBO_DYNA_FULL_SHELL_RECORD_TESTS OFF CACHE BOOL "Use owning record gate")
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../full_shell" "${CMAKE_CURRENT_BINARY_DIR}/frame-archive-records")
endif()
add_library(robo_dyna_physical_frame_archive STATIC
  "${CMAKE_CURRENT_LIST_DIR}/ArchivePrepare.cpp" "${CMAKE_CURRENT_LIST_DIR}/ArchiveWrite.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/FrameIOChecks.cpp")
target_link_libraries(robo_dyna_physical_frame_archive PUBLIC robo_dyna_full_shell_source_bundle robo_dyna_parent_activity)
target_include_directories(robo_dyna_physical_frame_archive PRIVATE "${ROBO_DYNA_TL_ROOT}")
target_compile_features(robo_dyna_physical_frame_archive PUBLIC cxx_std_17)
target_compile_options(robo_dyna_physical_frame_archive PRIVATE -fno-fast-math -ffp-contract=off)
