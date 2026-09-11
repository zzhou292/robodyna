include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../physical_attachments/VehiclePhysicalAttachments.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/ConnectivityValues.cmake")
add_library(robo_dyna_vehicle_connectivity STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp" "${CMAKE_CURRENT_LIST_DIR}/Relations.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Shells.cpp" "${CMAKE_CURRENT_LIST_DIR}/Elements.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Constraints.cpp" "${CMAKE_CURRENT_LIST_DIR}/Prepare.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Report.cpp")
target_link_libraries(robo_dyna_vehicle_connectivity PUBLIC
  robo_dyna_connectivity_values robo_dyna_vehicle_physical_attachments)
target_compile_features(robo_dyna_vehicle_connectivity PUBLIC cxx_std_17)
