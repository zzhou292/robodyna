if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "LAW90 native force qualifies GNU Fortran")
endif()
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE result)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "LAW90 force donor authentication failed")
endif()
add_custom_target(law90_solid18_force_verify_native
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
add_library(law90_solid18_force_native STATIC NativeCaller.F90 NativeViscosity.F NativeDensity.F NativeTags.F
  NativeAssembly.F NativeReduce.F NativeLength.F NativeWorld.F NativePacket.F90 NativeForce.F90
  "${prepared}/elbuftag_mod.F" "${prepared}/constant_mod.F" "${prepared}/precision_mod.F90"
  "${prepared}/s8efint3.F" "${prepared}/s8efmoy3.F" "${prepared}/s8etotsh10.F" "${prepared}/srrota3.F")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
file(MAKE_DIRECTORY "${modules}")
set_target_properties(law90_solid18_force_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_include_directories(law90_solid18_force_native PRIVATE "${prepared}" "${modules}"
  "${CMAKE_CURRENT_BINARY_DIR}/../kinematics_native/engine-modules")
target_compile_definitions(law90_solid18_force_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(law90_solid18_force_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
target_link_libraries(law90_solid18_force_native PUBLIC law90_point_native law90_preparation_native law90_solid18_current_native)
add_dependencies(law90_solid18_force_native law90_solid18_force_verify_native law90_solid18_current_native)
add_test(NAME law90_solid18_force_native_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
