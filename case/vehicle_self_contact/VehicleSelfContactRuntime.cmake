include_guard(GLOBAL)

include("${CMAKE_CURRENT_LIST_DIR}/VehicleSelfContactSetup.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_dynamics/VehiclePhysicalDynamics.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/SelfContactTransaction.cmake")

add_library(robo_dyna_vehicle_self_contact_startup STATIC
  "${CMAKE_CURRENT_LIST_DIR}/RuntimeBudget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VehicleSelfContactStartup.cpp")
target_link_libraries(robo_dyna_vehicle_self_contact_startup PUBLIC
  robo_dyna_vehicle_self_contact_setup
  robo_dyna_vehicle_physical_dynamics
  tl_self_contact_transaction)
target_compile_features(robo_dyna_vehicle_self_contact_startup PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_self_contact_startup PRIVATE
  -fno-fast-math -ffp-contract=off)

include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_wall/VehicleWallStartup.cmake")
add_library(robo_dyna_vehicle_self_contact_runtime STATIC
  "${CMAKE_CURRENT_LIST_DIR}/runtime/Operations.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/runtime/Stages.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/runtime/Prepare.cpp")
target_link_libraries(robo_dyna_vehicle_self_contact_runtime PUBLIC
  robo_dyna_vehicle_self_contact_startup
  robo_dyna_vehicle_loaded_wall)
target_compile_features(robo_dyna_vehicle_self_contact_runtime PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_self_contact_runtime PRIVATE
  -fno-fast-math -ffp-contract=off)
