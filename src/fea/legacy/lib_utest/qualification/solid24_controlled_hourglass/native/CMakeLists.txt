set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
file(MAKE_DIRECTORY "${modules}")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" COMMAND_ERROR_IS_FATAL ANY)
add_custom_target(h24_adapter_verify_native COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
set(ic1 "${CMAKE_CURRENT_SOURCE_DIR}/../../solid24_icontrol/native")
set(leaves szhour_ctl.F shour_ctl.F90 sdistror_ini.F90 scre_sig3.F s8for_distor.F
  sfor_visn8.F sfor_n2s4.F sfor_4n2s4.F90 sfor_ns2s4.F90 ssort_n4.F sfor_n2s3.F
  mvsiz_mod.F90 scumu3.F ControlledInterfaces.F NativeCaller.F90 NativeMaterialForces.F)
set(sources)
foreach(leaf IN LISTS leaves)
  list(APPEND sources "${prepared}/${leaf}")
endforeach()
add_library(h24_adapter_native STATIC Observers.F90 "${ic1}/NativeSlots.F90"
  "${ic1}/NativeObservers.F90" "${ic1}/NativeHourglass.F" "${ic1}/NativeDistortion.F"
  "${ic1}/NativeAssembly.F" "${ic1}/NativeParameterProbe.F90" ${sources})
add_dependencies(h24_adapter_native h24_adapter_verify_native heph_force_native)
set_target_properties(h24_adapter_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
get_target_property(heph_modules heph_force_native Fortran_MODULE_DIRECTORY)
target_include_directories(h24_adapter_native PRIVATE "${prepared}" "${modules}"
  "$<TARGET_PROPERTY:heph_force_native,Fortran_MODULE_DIRECTORY>"
  "${heph_modules}/../prepared")
target_compile_definitions(h24_adapter_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(h24_adapter_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
target_link_libraries(h24_adapter_native PUBLIC heph_force_native)
add_test(NAME h24_adapter_native_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
