if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "HEPH native force currently qualifies GNU Fortran only")
endif()
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE status)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "HEPH native force donor preparation failed")
endif()
file(MAKE_DIRECTORY "${modules}")
add_custom_target(heph_force_verify_native COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
set(leaves szderi3 szhour3 szetfac sordeft3 sztorth3 sgcoor3 sdefot3
  sdefo3 sdlen3 slen sfint3 srepiso3 sortho3 srrota3 schkjabt3)
set(sources)
foreach(leaf IN LISTS leaves)
  list(APPEND sources "${prepared}/${leaf}.F")
endforeach()
add_library(heph_force_native STATIC NativeContext.F90 NativeGlobals.F NativeGeometry.F
  NativeReduce.F NativeCaller.F90 "${prepared}/NativeInterfaces.F" ${sources}
  "${prepared}/constant_mod.F" "${prepared}/precision_mod.F90" "${prepared}/element_mod.F90")
add_dependencies(heph_force_native heph_force_verify_native)
set_target_properties(heph_force_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_include_directories(heph_force_native PRIVATE "${prepared}" "${modules}")
target_compile_definitions(heph_force_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(heph_force_native PRIVATE -cpp -ffixed-line-length-none
  -ffree-line-length-none -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
target_link_libraries(heph_force_native PUBLIC law42_solid_caller_native solid24_reference_native)
add_test(NAME heph_force_native_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check)
