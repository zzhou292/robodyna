if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "TYPE45 native packet currently qualifies GNU Fortran only")
endif()
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE status)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "TYPE45 native source preparation failed")
endif()
file(MAKE_DIRECTORY "${modules}")
add_custom_target(type45_verify_native COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
add_library(type45_joint_native STATIC Context.F90 Interfaces.F90 NativeGlobals.F
  NativeAutomatic.F NativeStartup.F90 NativeStep.F90
  "${prepared}/constant_mod.F" "${prepared}/rini45.F" "${prepared}/rini45_rb.F"
  "${prepared}/rskew33.F" "${prepared}/ruser33.F" "${prepared}/rdtime33.F" "${prepared}/rcum33.F")
add_dependencies(type45_joint_native type45_verify_native)
set_target_properties(type45_joint_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_include_directories(type45_joint_native PRIVATE "${prepared}" "${modules}" "${CMAKE_CURRENT_SOURCE_DIR}/context")
target_compile_options(type45_joint_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
add_test(NAME type45_native_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
