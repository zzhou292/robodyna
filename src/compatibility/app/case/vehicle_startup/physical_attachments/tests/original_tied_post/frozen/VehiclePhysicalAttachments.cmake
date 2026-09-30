include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../physical_model/VehiclePhysicalModel.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../TiedCinWitnessRoster.cmake")
add_library(robo_dyna_vehicle_physical_attachments STATIC "${CMAKE_CURRENT_LIST_DIR}/VehiclePhysicalAttachments.cpp")
target_link_libraries(robo_dyna_vehicle_physical_attachments PUBLIC
  robo_dyna_vehicle_physical_model robo_dyna_tied_cin_witness_roster)
target_compile_features(robo_dyna_vehicle_physical_attachments PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_physical_attachments PRIVATE -fno-fast-math -ffp-contract=off)
