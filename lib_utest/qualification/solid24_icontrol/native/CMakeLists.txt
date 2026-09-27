set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
file(MAKE_DIRECTORY "${modules}")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" COMMAND_ERROR_IS_FATAL ANY)
add_custom_target(ic1_verify_native COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
set(leaves szhour_ctl.F shour_ctl.F90 sdistror_ini.F90 scre_sig3.F s8for_distor.F
  sfor_visn8.F sfor_n2s4.F sfor_4n2s4.F90 sfor_ns2s4.F90 ssort_n4.F sfor_n2s3.F
  mvsiz_mod.F90 scumu3.F)
set(sources)
foreach(leaf IN LISTS leaves)
  list(APPEND sources "${prepared}/${leaf}")
endforeach()
add_library(solid24_ic1_native STATIC NativeSlots.F90 NativeObservers.F90 NativeCaller.F90
  NativeHourglass.F NativeMaterialForces.F NativeDistortion.F NativeAssembly.F
  "${prepared}/ControlledInterfaces.F" ${sources})
add_dependencies(solid24_ic1_native ic1_verify_native heph_force_native)
set_target_properties(solid24_ic1_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_include_directories(solid24_ic1_native PRIVATE "${prepared}" "${modules}"
  "$<TARGET_PROPERTY:heph_force_native,Fortran_MODULE_DIRECTORY>"
  "${CMAKE_BINARY_DIR}/legacy_heph/native/prepared")
target_compile_definitions(solid24_ic1_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(solid24_ic1_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
target_link_libraries(solid24_ic1_native PUBLIC heph_force_native)
add_test(NAME solid24_ic1_native_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
