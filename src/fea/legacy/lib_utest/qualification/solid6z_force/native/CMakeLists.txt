if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Selected corrected S6Z native force currently qualifies GNU Fortran only")
endif()
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE status)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "S6Z native force source preparation failed")
endif()
file(MAKE_DIRECTORY "${modules}")
add_custom_target(solid6z_force_verify_native COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
add_library(solid6z_force_native STATIC NativeContext.F90 NativeGlobals.F
  NativeCoordinates.F90 NativeCurrent.F90 NativeResultants.F90 NativeForce.F90 NativeInitial.F90
  "${prepared}/constant_mod.F" "${prepared}/precision_mod.F90"
  "${prepared}/s6zrcoor3.F90" "${prepared}/s6zderi3.F90" "${prepared}/s6zderito3.F90"
  "${prepared}/s6zdefot3.F90" "${prepared}/s6zdefc3.F90" "${prepared}/s6zdefo3.F90"
  "${prepared}/s6zhourg3.F90" "${prepared}/s6zfint3.F90" "${prepared}/s6zrrota3.F90"
  "${prepared}/sgcoor3.F" "${prepared}/sdlen3.F" "${prepared}/slen.F"
  "${prepared}/srepiso3.F" "${prepared}/sortho3.F" "${prepared}/srrota3.F"
  "${prepared}/szetfac.F" "${prepared}/sztorth3.F" "${prepared}/sordeft3.F"
  "${prepared}/schkjabt3.F")
add_dependencies(solid6z_force_native solid6z_force_verify_native)
set_target_properties(solid6z_force_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_include_directories(solid6z_force_native PRIVATE "${prepared}" "${modules}")
target_compile_definitions(solid6z_force_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(solid6z_force_native PRIVATE -cpp -ffixed-line-length-none
  -ffree-line-length-none -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
target_link_libraries(solid6z_force_native PUBLIC law42_solid_caller_native)
add_test(NAME solid6z_force_native_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
