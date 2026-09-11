if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Solid18 native force currently qualifies GNU Fortran only")
endif()
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE status)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "Solid18 force donor preparation failed")
endif()
file(MAKE_DIRECTORY "${modules}")
add_custom_target(solid18_force_verify_native
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --output "${prepared}" --check VERBATIM)
set(leaves s8ederic3 s8ejacip3 s8ederipr3 s8ederig3 s8eselecsh s8ederi_bij
  s8edefo3 s8efint3 s8efmoy3 s8eprst_ini schkjabt3 srepiso3 sortho3 srrota3
  s8zsigp3 s8esigp3i s8ederish2 s8ea2bp8)
set(native_sources)
foreach(leaf IN LISTS leaves)
  list(APPEND native_sources "${prepared}/${leaf}.F")
endforeach()
add_library(solid18_force_native STATIC NativeContext.F90 NativeGeometry.F NativeSelection.F
  NativePoint.F NativeViscosity.F NativeMaterial.F90 NativeReduce.F NativeCaller.F
  ${native_sources} "${prepared}/constant_mod.F" "${prepared}/precision_mod.F90")
add_dependencies(solid18_force_native solid18_force_verify_native)
set_target_properties(solid18_force_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_compile_definitions(solid18_force_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(solid18_force_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
target_include_directories(solid18_force_native PRIVATE "${prepared}" "${modules}")
target_link_libraries(solid18_force_native PUBLIC solid_law36_native)
add_test(NAME solid18_force_native_identity
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --output "${prepared}" --check)
