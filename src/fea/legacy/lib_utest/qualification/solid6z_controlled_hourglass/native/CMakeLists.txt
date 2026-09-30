set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
file(MAKE_DIRECTORY "${modules}")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" COMMAND_ERROR_IS_FATAL ANY)
add_custom_target(s6_control_verify_native COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
add_library(s6_control_native STATIC Observers.F90 Assembly.F90
  "${prepared}/s6chour_ctl.F90" "${prepared}/s6cumu3.F"
  "${prepared}/NativeCaller.F90" "${prepared}/NativeResultants.F90" "${prepared}/NativeInitial.F90")
add_dependencies(s6_control_native s6_control_verify_native solid6z_force_native controlled_hourglass_native)
set_target_properties(s6_control_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_include_directories(s6_control_native PRIVATE "${modules}" "${prepared}"
  "$<TARGET_PROPERTY:solid6z_force_native,Fortran_MODULE_DIRECTORY>"
  "$<TARGET_PROPERTY:controlled_hourglass_native,Fortran_MODULE_DIRECTORY>"
  "${CMAKE_BINARY_DIR}/raw_hourglass/prepared")
target_compile_definitions(s6_control_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(s6_control_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
target_link_libraries(s6_control_native PUBLIC solid6z_force_native controlled_hourglass_native solid6z_reference_native)
add_test(NAME s6_control_native_identity COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
