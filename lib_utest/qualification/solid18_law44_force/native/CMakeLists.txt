set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE status)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "Rear force donor preparation failed")
endif()
file(MAKE_DIRECTORY "${modules}")
add_custom_target(rear18_force_verify_native
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --output "${prepared}" --check VERBATIM)
add_library(rear18_force_native STATIC NativeInterfaces.F90 NativeEntryInterface.F90 NativeEntries.F NativeGeometry.F NativeControl.F
  NativePoint.F NativeMaterial.F90 NativeReduce.F NativeCaller.F
  "${prepared}/degenes8.F" "${prepared}/s8edefoc3.F" "${prepared}/s8zfintp3.F" "${prepared}/intab.F")
add_dependencies(rear18_force_native rear18_force_verify_native solid18_force_native solid18_reference_native)
set_target_properties(rear18_force_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_compile_definitions(rear18_force_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(rear18_force_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
target_include_directories(rear18_force_native PRIVATE "${prepared}" "${modules}"
  "${CMAKE_CURRENT_BINARY_DIR}/../legacy-force/prepared" "${CMAKE_CURRENT_BINARY_DIR}/../legacy-force/modules"
  "${CMAKE_CURRENT_BINARY_DIR}/../reference-native/modules")
target_link_libraries(rear18_force_native PUBLIC solid18_force_native solid18_reference_native solid_law44_native)
add_test(NAME rear18_force_native_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
