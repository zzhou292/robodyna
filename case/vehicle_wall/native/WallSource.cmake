include_guard(GLOBAL)
get_filename_component(_wall_app "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${_wall_app}/modelio/physical_domain/VehiclePhysicalDomain.cmake")
include("${_wall_app}/modelio/native_spring_ids/NativeSpringIds.cmake")
include("${_wall_app}/case/vehicle_wall/VehicleWallValues.cmake")
include("${_wall_app}/case/CanonicalWallArtifacts.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25ShellSource.cmake")
add_library(robo_dyna_native_envelope_wall_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/WallSource.cpp" "${CMAKE_CURRENT_LIST_DIR}/Namespace.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Geometry.cpp" "${CMAKE_CURRENT_LIST_DIR}/CombinedDomain.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Digest.cpp")
target_link_libraries(robo_dyna_native_envelope_wall_source PUBLIC
  robo_dyna_vehicle_physical_domain robo_dyna_native_spring_ids
  robo_dyna_vehicle_wall_values robo_dyna_canonical_wall_artifacts tl_radioss_type25_shell_source)
target_include_directories(robo_dyna_native_envelope_wall_source PUBLIC "${_wall_app}")
target_compile_features(robo_dyna_native_envelope_wall_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_native_envelope_wall_source PRIVATE -fno-fast-math -ffp-contract=off)
