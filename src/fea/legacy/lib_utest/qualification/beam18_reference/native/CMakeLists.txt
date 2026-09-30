set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
file(MAKE_DIRECTORY "${modules}")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE status)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "Beam18 source authentication failed")
endif()
add_library(beam18_native STATIC Context.F90 NativeReference.F
  "${prepared}/constant_mod.F" "${prepared}/defbeam_sect.F"
  "${prepared}/pcoori.F" "${prepared}/peveci.F" "${prepared}/pmass.F")
set_target_properties(beam18_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_include_directories(beam18_native PRIVATE "${prepared}" "${modules}" "${CMAKE_CURRENT_SOURCE_DIR}/context")
target_compile_options(beam18_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
add_test(NAME beam18_native_source_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
