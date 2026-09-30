if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "LAW36 native point currently qualifies GNU Fortran only")
endif()
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE status)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "LAW36 donor preparation failed")
endif()
file(MAKE_DIRECTORY "${modules}")
add_custom_target(solid_law36_verify_native
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --output "${prepared}" --check VERBATIM)
add_library(solid_law36_native STATIC NativeSetup.F NativePoint.F90 NativeCaller.F90
  "${prepared}/sigeps36.F" "${prepared}/m36iter_imp.F" "${prepared}/mstrain_rate.F"
  "${prepared}/vinter.F" "${prepared}/constant_mod.F" "${prepared}/precision_mod.F90")
add_dependencies(solid_law36_native solid_law36_verify_native)
set_target_properties(solid_law36_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_compile_definitions(solid_law36_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(solid_law36_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
target_include_directories(solid_law36_native PRIVATE "${prepared}" "${modules}"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../native/qeph/original/engine/share/spe_inc"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../native/qeph/original/engine/share/includes"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../native/qeph/original/engine/share/r8")
add_test(NAME solid_law36_native_identity
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --output "${prepared}" --check)
