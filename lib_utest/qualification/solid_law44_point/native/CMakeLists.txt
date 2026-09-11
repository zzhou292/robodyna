if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Selected native LAW44 gate requires GNU Fortran")
endif()
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE status)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "LAW44 source authentication failed")
endif()
file(MAKE_DIRECTORY "${modules}")
add_library(solid_law44_native STATIC NativeSetup.F NativeInterface.F NativePoint.F90
  "${prepared}/sigeps44.F" "${prepared}/mstrain_rate.F" "${prepared}/vinter.F"
  "${prepared}/constant_mod.F" "${prepared}/precision_mod.F90")
set_target_properties(solid_law44_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_compile_definitions(solid_law44_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(solid_law44_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
target_include_directories(solid_law44_native PRIVATE "${prepared}" "${modules}")
add_custom_target(solid_law44_verify_native
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --output "${prepared}" --check VERBATIM)
add_dependencies(solid_law44_native solid_law44_verify_native)
add_test(NAME solid_law44_native_identity
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --output "${prepared}" --check)
