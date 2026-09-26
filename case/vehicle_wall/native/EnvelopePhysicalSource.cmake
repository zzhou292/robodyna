include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/WallSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_startup/physical_model/VehiclePhysicalModel.cmake")
add_library(robo_dyna_envelope_physical_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/EnvelopePhysicalSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/physical/Packing.cpp")
target_link_libraries(robo_dyna_envelope_physical_source PUBLIC
  robo_dyna_native_envelope_wall_source robo_dyna_vehicle_physical_model)
target_compile_features(robo_dyna_envelope_physical_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_envelope_physical_source PRIVATE -fno-fast-math -ffp-contract=off)
