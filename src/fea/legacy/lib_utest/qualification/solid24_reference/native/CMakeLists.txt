if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Solid24 native startup currently qualifies GNU Fortran only")
endif()
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE result)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "Solid24 native source preparation failed")
endif()
file(MAKE_DIRECTORY "${modules}")
add_custom_target(solid24_reference_verify_native
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" --check VERBATIM)
add_library(solid24_reference_native STATIC NativeReference.F NativeContext.F90
  "${CMAKE_CURRENT_SOURCE_DIR}/../../solid18_reference/native/NativeMassContext.F90"
  "${prepared}/constant_mod.F" "${prepared}/precision_mod.F90" "${prepared}/element_mod.F90"
  "${prepared}/scoor3.F" "${prepared}/srcoor3.F" "${prepared}/sjacidp.F"
  "${prepared}/checksvolume.F" "${prepared}/srepiso3.F" "${prepared}/sortho3.F"
  "${prepared}/szderi3.F" "${prepared}/sdlen3.F" "${prepared}/slen.F" "${prepared}/smass3.F")
add_dependencies(solid24_reference_native solid24_reference_verify_native)
set_target_properties(solid24_reference_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_include_directories(solid24_reference_native PRIVATE "${prepared}" "${modules}")
target_compile_definitions(solid24_reference_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(solid24_reference_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
add_test(NAME solid24_reference_native_identity
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" --check)
