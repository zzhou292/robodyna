set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE status)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "LAW42 source preparation failed")
endif()
file(MAKE_DIRECTORY "${modules}")
add_custom_target(law42_verify_native COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
add_library(law42_native STATIC NativePoint.F90 NativeGlobals.F
  "${prepared}/sigeps42.F" "${prepared}/VALPVEC_V.F" "${prepared}/VALPVECDP_V.F"
  "${prepared}/prodAAT.F" "${prepared}/prodmat.F" "${prepared}/precision.c"
  "${prepared}/constant_mod.F" "${prepared}/precision_mod.F90")
add_dependencies(law42_native law42_verify_native)
set_target_properties(law42_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_include_directories(law42_native PRIVATE "${prepared}" "${modules}")
target_compile_definitions(law42_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(law42_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace>"
  -fno-fast-math -ffp-contract=off)
add_test(NAME law42_native_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
