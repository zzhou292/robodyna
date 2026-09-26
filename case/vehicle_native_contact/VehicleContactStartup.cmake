include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SourceAdmission.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/FieldPacking.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_dynamics/VehiclePhysicalDynamics.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25InitialState.cmake")
add_library(robo_dyna_vehicle_native_contact STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Config.cpp" "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/TiedRemovalSource.cpp" "${CMAKE_CURRENT_LIST_DIR}/InitialModel.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/InterfaceFields.cpp" "${CMAKE_CURRENT_LIST_DIR}/RuntimeSources.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Prepare.cpp" "${CMAKE_CURRENT_LIST_DIR}/Initialize.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Census.cpp")
target_link_libraries(robo_dyna_vehicle_native_contact PUBLIC
  robo_dyna_native_vehicle_source_admission robo_dyna_native_vehicle_field_packing
  robo_dyna_vehicle_physical_dynamics tl_radioss_type25_initial_state CUDA::cudart)
target_compile_features(robo_dyna_vehicle_native_contact PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_native_contact PRIVATE -fno-fast-math -ffp-contract=off)
