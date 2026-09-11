if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "LAW42 native caller currently qualifies GNU Fortran only")
endif()
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE status)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "LAW42 caller source preparation failed")
endif()
file(MAKE_DIRECTORY "${modules}")
add_custom_target(law42_caller_verify_native COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
add_library(law42_solid_caller_native STATIC NativeCaller.F90 NativeInitial.F90 NativeViscosity.F
  "${prepared}/constant_mod.F" "${prepared}/precision_mod.F90")
add_dependencies(law42_solid_caller_native law42_caller_verify_native)
set_target_properties(law42_solid_caller_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_include_directories(law42_solid_caller_native PRIVATE "${prepared}" "${modules}")
target_compile_definitions(law42_solid_caller_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(law42_solid_caller_native PRIVATE -cpp -ffixed-line-length-none
  -ffree-line-length-none -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
target_link_libraries(law42_solid_caller_native PUBLIC law42_native)
add_test(NAME law42_caller_native_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
