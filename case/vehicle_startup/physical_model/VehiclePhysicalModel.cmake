include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../../modelio/solid_control_packets/NativePacketSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../VehicleShellBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../../modelio/physical_domain/VehiclePhysicalDomain.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../../modelio/point_mass/VehiclePointMassSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../../modelio/type25/VehicleType25Source.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/constraints/NodalRigidAssemblyBinding.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/assembly/Beam18NodeContributions.cmake")
add_subdirectory("${ROBO_DYNA_TL_ROOT}/lib_src/elements/solids" "${CMAKE_CURRENT_BINARY_DIR}/tl_solid_model")
add_library(robo_dyna_vehicle_physical_model STATIC
  "${CMAKE_CURRENT_LIST_DIR}/VehiclePhysicalModel.cpp" "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Components.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StructuralBeams.cpp" "${CMAKE_CURRENT_LIST_DIR}/BeamModel.cpp" "${CMAKE_CURRENT_LIST_DIR}/SolidModel.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SolidControls.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PlainGroups.cpp")
target_link_libraries(robo_dyna_vehicle_physical_model PUBLIC robo_dyna_native_solid_packets robo_dyna_vehicle_shell_binding
  robo_dyna_vehicle_physical_domain robo_dyna_vehicle_point_mass_source robo_dyna_vehicle_type25_source
  tl_solid_model tl_beam18_node_contributions tl_nodal_rigid_assembly_binding)
target_compile_features(robo_dyna_vehicle_physical_model PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_physical_model PRIVATE -fno-fast-math -ffp-contract=off)
