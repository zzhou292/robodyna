# HOST-only binding qualification. No resident shell/contact or CUDA dependency.
get_filename_component(vehicle_shell_tl_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${vehicle_shell_tl_root}/lib_src/elements/ShellBatchPlasticityBinding.cmake")
add_executable(vehicle_shell_host_check
  "${CMAKE_CURRENT_LIST_DIR}/VehicleShellAdmissionTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VehicleShellSourceSizeTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../host_shell_collection/AllocationFailure.cpp")
target_include_directories(vehicle_shell_host_check PRIVATE "${vehicle_shell_tl_root}")
target_link_libraries(vehicle_shell_host_check PRIVATE tl_shell_batch_plasticity_binding GTest::gtest_main)
target_compile_options(vehicle_shell_host_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_shell_host_small COMMAND vehicle_shell_host_check --gtest_filter=VehicleShellHost.*)
add_test(NAME vehicle_shell_host_source_size COMMAND vehicle_shell_host_check --gtest_filter=VehicleShellHostSourceSize.*)
set_tests_properties(vehicle_shell_host_small PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
set_tests_properties(vehicle_shell_host_source_size PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
