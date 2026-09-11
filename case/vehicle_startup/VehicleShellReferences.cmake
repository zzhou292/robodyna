include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../modelio/vehicle_sections/VehicleSectionResolution.cmake")
# Native startup is header-only; no resident, owner, CUDA, or Fortran target.
add_library(robo_dyna_vehicle_native_reference STATIC "${CMAKE_CURRENT_LIST_DIR}/NativeReference.cpp")
target_link_libraries(robo_dyna_vehicle_native_reference PUBLIC robo_dyna_source_assembly)
target_compile_features(robo_dyna_vehicle_native_reference PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_native_reference PRIVATE -fno-fast-math -ffp-contract=off)
add_library(robo_dyna_vehicle_shell_references STATIC
  "${CMAKE_CURRENT_LIST_DIR}/VehicleShellReferences.cpp" "${CMAKE_CURRENT_LIST_DIR}/ReferenceGeometry.cpp")
target_link_libraries(robo_dyna_vehicle_shell_references PUBLIC robo_dyna_vehicle_sections robo_dyna_vehicle_native_reference)
target_compile_features(robo_dyna_vehicle_shell_references PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_shell_references PRIVATE -fno-fast-math -ffp-contract=off)
