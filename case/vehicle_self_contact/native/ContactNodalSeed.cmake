include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_startup/joints/VehicleJointModel.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/TopologyAssessment.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../../modelio/native_spring_ids/NativeSpringIds.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25ShellSource.cmake")
add_library(robo_dyna_vehicle_contact_nodal_seed STATIC
  "${CMAKE_CURRENT_LIST_DIR}/ContactNodalSeed.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_seed/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_seed/Context.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_seed/SolidValues.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_seed/DirectValues.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_seed/ShellValues.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_seed/Digest.cpp")
target_link_libraries(robo_dyna_vehicle_contact_nodal_seed PUBLIC
  robo_dyna_vehicle_joint_model robo_dyna_native_spring_ids
  robo_dyna_native_topology_assessment tl_radioss_type25_shell_source)
target_compile_features(robo_dyna_vehicle_contact_nodal_seed PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_contact_nodal_seed PRIVATE -fno-fast-math -ffp-contract=off)
