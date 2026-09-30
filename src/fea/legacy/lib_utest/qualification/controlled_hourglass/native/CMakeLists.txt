if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Controlled leaf oracle qualifies GNU Fortran only")
endif()
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
file(MAKE_DIRECTORY "${modules}")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" COMMAND_ERROR_IS_FATAL ANY)
add_custom_target(controlled_hourglass_verify_native COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
add_library(controlled_hourglass_native STATIC Leaf.F90 Observer.F90
  "${CMAKE_CURRENT_SOURCE_DIR}/../../solid24_icontrol/native/NativeSlots.F90"
  "${prepared}/constant_mod.F" "${prepared}/precision_mod.F90"
  "${prepared}/mvsiz_mod.F90" "${prepared}/shour_ctl.F90")
add_dependencies(controlled_hourglass_native controlled_hourglass_verify_native)
set_target_properties(controlled_hourglass_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_include_directories(controlled_hourglass_native PRIVATE "${prepared}" "${modules}")
target_compile_definitions(controlled_hourglass_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(controlled_hourglass_native PRIVATE -cpp -ffixed-line-length-none
  -ffree-line-length-none -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
add_test(NAME controlled_hourglass_native_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
