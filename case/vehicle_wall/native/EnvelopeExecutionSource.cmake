include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/EnvelopePhysicalSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_startup/shell_execution/VehicleShellExecution.cmake")
add_library(robo_dyna_envelope_execution_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/EnvelopeExecutionSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/execution/Packing.cpp")
target_link_libraries(robo_dyna_envelope_execution_source PUBLIC
  robo_dyna_envelope_physical_source robo_dyna_vehicle_shell_execution)
target_compile_features(robo_dyna_envelope_execution_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_envelope_execution_source PRIVATE -fno-fast-math -ffp-contract=off)
