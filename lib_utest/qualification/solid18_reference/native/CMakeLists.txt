if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Solid18 native reference currently qualifies GNU Fortran only")
endif()
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE status)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "Solid18 donor preparation failed")
endif()
file(MAKE_DIRECTORY "${modules}")
add_custom_target(solid18_reference_verify_native
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --output "${prepared}" --check VERBATIM)
add_library(solid18_reference_native STATIC NativeContext.F90 NativeReference.F
  "${prepared}/checksvolume.F" "${prepared}/srepiso3.F" "${prepared}/sortho3.F"
  "${prepared}/basisf.F" "${prepared}/basis8.F" "${prepared}/sderi3b.F"
  "${prepared}/smass3b.F" "${prepared}/element_mod.F90"
  "${prepared}/constant_mod.F" "${prepared}/precision_mod.F90")
add_dependencies(solid18_reference_native solid18_reference_verify_native)
set_target_properties(solid18_reference_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_compile_definitions(solid18_reference_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(solid18_reference_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
target_include_directories(solid18_reference_native PRIVATE "${prepared}" "${modules}")
add_test(NAME solid18_reference_native_identity
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --output "${prepared}" --check)
