include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/VehicleSelfContactSetup.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_dynamics/VehiclePhysicalDynamics.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/SelfContactBroadphase.cmake")
add_library(robo_dyna_vehicle_self_contact_initial_census STATIC
  "${CMAKE_CURRENT_LIST_DIR}/VehicleSelfContactInitialCensus.cpp")
target_link_libraries(robo_dyna_vehicle_self_contact_initial_census PUBLIC
  robo_dyna_vehicle_self_contact_setup
  robo_dyna_vehicle_physical_dynamics
  tl_self_contact_broadphase)
target_compile_features(robo_dyna_vehicle_self_contact_initial_census
  PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_self_contact_initial_census PRIVATE
  -fno-fast-math -ffp-contract=off)
