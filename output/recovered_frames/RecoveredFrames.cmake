include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../physical_run/PhysicalRun.cmake")
add_library(robo_dyna_recovered_frames STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Descriptor.cpp" "${CMAKE_CURRENT_LIST_DIR}/Samples.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Files.cpp" "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Source.cpp" "${CMAKE_CURRENT_LIST_DIR}/Prepare.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Replay.cpp" "${CMAKE_CURRENT_LIST_DIR}/Environment.cpp")
target_link_libraries(robo_dyna_recovered_frames PUBLIC robo_dyna_physical_run_records)
target_include_directories(robo_dyna_recovered_frames PRIVATE "${ROBO_DYNA_TL_ROOT}")
target_compile_features(robo_dyna_recovered_frames PUBLIC cxx_std_17)
target_compile_options(robo_dyna_recovered_frames PRIVATE -fno-fast-math -ffp-contract=off)
