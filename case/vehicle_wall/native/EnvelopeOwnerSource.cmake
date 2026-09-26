include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/EnvelopeExecutionSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_startup/TiedCinWitnessRoster.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_startup/joints/VehicleJointModel.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_runtime/Values.cmake")
add_library(robo_dyna_envelope_owner_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/EnvelopeOwnerSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/owner_source/Admission.cpp")
target_link_libraries(robo_dyna_envelope_owner_source PUBLIC robo_dyna_envelope_execution_source
  robo_dyna_tied_cin_witness_roster robo_dyna_vehicle_joint_model robo_dyna_vehicle_runtime_values)
target_compile_features(robo_dyna_envelope_owner_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_envelope_owner_source PRIVATE -fno-fast-math -ffp-contract=off)
