include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/VehicleShellReferences.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/ShellBatchBinding.cmake")
add_library(robo_dyna_vehicle_shell_binding STATIC "${CMAKE_CURRENT_LIST_DIR}/VehicleShellBinding.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/shell_binding/Budget.cpp" "${CMAKE_CURRENT_LIST_DIR}/shell_binding/Inputs.cpp")
target_link_libraries(robo_dyna_vehicle_shell_binding PUBLIC robo_dyna_vehicle_shell_references tl_shell_batch_binding)
target_compile_features(robo_dyna_vehicle_shell_binding PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_shell_binding PRIVATE -fno-fast-math -ffp-contract=off)
