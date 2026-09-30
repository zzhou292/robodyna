if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "LAW90 native preparation qualifies GNU Fortran")
endif()
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE result)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "LAW90 complete donor authentication failed")
endif()
file(MAKE_DIRECTORY "${modules}")
add_library(law90_preparation_native STATIC NativeContext.F90 NativeSetup.F
  "${prepared}/constant_mod.F" "${prepared}/precision_mod.F90"
  "${prepared}/func_slope.F" "${prepared}/law90_upd.F" "${prepared}/vinter.F")
set_target_properties(law90_preparation_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_include_directories(law90_preparation_native PRIVATE "${prepared}" "${modules}")
target_compile_definitions(law90_preparation_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(law90_preparation_native PRIVATE -cpp -ffixed-line-length-none
  -ffree-line-length-none -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
add_test(NAME law90_native_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
