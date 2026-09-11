include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../physical_model/VehiclePhysicalModel.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/assembly/ShellPhysicalBinding.cmake")
add_library(robo_dyna_vehicle_shell_execution_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Packing.cpp" "${CMAKE_CURRENT_LIST_DIR}/Curves.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Resource.cpp")
target_include_directories(robo_dyna_vehicle_shell_execution_values PUBLIC "${ROBO_DYNA_TL_ROOT}")
target_link_libraries(robo_dyna_vehicle_shell_execution_values PUBLIC robo_dyna_artifact_io)
target_compile_features(robo_dyna_vehicle_shell_execution_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_shell_execution_values PRIVATE -fno-fast-math -ffp-contract=off)

add_library(robo_dyna_vehicle_shell_execution STATIC
  "${CMAKE_CURRENT_LIST_DIR}/VehicleShellExecution.cpp" "${CMAKE_CURRENT_LIST_DIR}/SourceChecks.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp")
target_link_libraries(robo_dyna_vehicle_shell_execution PUBLIC robo_dyna_vehicle_shell_execution_values
  robo_dyna_vehicle_physical_model tl_shell_physical_binding)
target_compile_features(robo_dyna_vehicle_shell_execution PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_shell_execution PRIVATE -fno-fast-math -ffp-contract=off)
