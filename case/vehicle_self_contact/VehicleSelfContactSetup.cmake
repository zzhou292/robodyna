include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/VehicleSelfContactValues.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_startup/shell_execution/VehicleShellExecution.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_startup/physical_attachments/VehiclePhysicalAttachments.cmake")
add_library(robo_dyna_vehicle_self_contact_setup STATIC
  "${CMAKE_CURRENT_LIST_DIR}/VehicleSelfContactSetup.cpp")
target_link_libraries(robo_dyna_vehicle_self_contact_setup PUBLIC
  robo_dyna_vehicle_self_contact_values
  robo_dyna_vehicle_shell_execution
  robo_dyna_vehicle_physical_attachments)
target_compile_features(robo_dyna_vehicle_self_contact_setup PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_self_contact_setup PRIVATE
  -fno-fast-math -ffp-contract=off)
