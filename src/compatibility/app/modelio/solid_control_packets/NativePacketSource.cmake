include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../solid_source/VehicleSolidSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../solid_control/SolidControlSource.cmake")
add_library(robo_dyna_native_solid_packets STATIC
  "${CMAKE_CURRENT_LIST_DIR}/NativePacketSource.cpp" "${CMAKE_CURRENT_LIST_DIR}/Reader.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Binding.cpp" "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp")
target_link_libraries(robo_dyna_native_solid_packets PUBLIC robo_dyna_vehicle_solid_source robo_dyna_solid_control_source)
target_compile_features(robo_dyna_native_solid_packets PUBLIC cxx_std_17)
target_compile_options(robo_dyna_native_solid_packets PRIVATE -fno-fast-math -ffp-contract=off)
