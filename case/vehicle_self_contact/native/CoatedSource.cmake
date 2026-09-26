include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TopologyAssessment.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_startup/physical_model/VehiclePhysicalModel.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../../modelio/source_assembly/NativeCoordinates.cmake")
if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_src/collision/radioss_type25/startup/CoatingOrientation.h" OR
   NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_src/collision/radioss_type25/startup/RolePolicy.h")
  message(FATAL_ERROR "V5 coated source requires TL coating-orientation and resolved-shell-startup capabilities; qualification is a separate owning gate")
endif()
add_library(robo_dyna_v5_coated_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/coated/Reader.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/coated/Membership.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/coated/Order.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/coated/Inputs.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/coated/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/coated/Digest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/coated/Assessment.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/coated/Document.cpp")
target_link_libraries(robo_dyna_v5_coated_source PUBLIC robo_dyna_vehicle_physical_model
  robo_dyna_native_topology_assessment robo_dyna_source_native_coordinates)
target_compile_features(robo_dyna_v5_coated_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_v5_coated_source PRIVATE -fno-fast-math -ffp-contract=off)
