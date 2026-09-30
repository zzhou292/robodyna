include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TiedCinWitnessRoster.cmake")
if(NOT TARGET tl_shell_batch_publication)
  message(FATAL_ERROR "CIN activity adapter requires the owning complete shell publication target")
endif()
add_library(robo_dyna_tied_cin_witness_activity STATIC
  "${CMAKE_CURRENT_LIST_DIR}/TiedCinWitnessActivity.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tied_cin_witness/ActivityCapture.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tied_cin_witness/ActivityUpload.cpp")
target_link_libraries(robo_dyna_tied_cin_witness_activity PUBLIC
  robo_dyna_tied_cin_witness_roster tl_shell_batch_publication)
target_compile_features(robo_dyna_tied_cin_witness_activity PUBLIC cxx_std_17)
target_compile_options(robo_dyna_tied_cin_witness_activity PRIVATE -fno-fast-math -ffp-contract=off)
