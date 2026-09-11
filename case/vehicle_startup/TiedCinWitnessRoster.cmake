include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TiedCinAttachments.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/VehicleShellBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/TiedCinWitnessValues.cmake")
add_library(robo_dyna_tied_cin_witness_roster STATIC "${CMAKE_CURRENT_LIST_DIR}/TiedCinWitnessRoster.cpp")
target_link_libraries(robo_dyna_tied_cin_witness_roster PUBLIC robo_dyna_tied_cin_witness_values
  robo_dyna_tied_cin_attachments robo_dyna_vehicle_shell_binding)
target_compile_features(robo_dyna_tied_cin_witness_roster PUBLIC cxx_std_17)
target_compile_options(robo_dyna_tied_cin_witness_roster PRIVATE -fno-fast-math -ffp-contract=off)
