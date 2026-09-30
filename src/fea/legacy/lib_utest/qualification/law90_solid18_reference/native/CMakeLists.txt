if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "LAW90 solid18 native reference qualifies GNU Fortran")
endif()
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE result)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "LAW90 solid18 donor authentication failed")
endif()
add_custom_target(law90_solid18_verify_native
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --output "${prepared}" --check VERBATIM)
add_library(law90_solid18_startup_native STATIC NativeStartContext.F90 NativeReference.F
  "${prepared}/starter/constant_mod.F" "${prepared}/starter/precision_mod.F90"
  "${prepared}/starter/element_mod.F90" "${prepared}/starter/checksvolume.F"
  "${prepared}/starter/srepiso3.F" "${prepared}/starter/sortho3.F"
  "${prepared}/starter/s8zderi3.F" "${prepared}/starter/sgsavini.F")
target_link_libraries(law90_solid18_startup_native PUBLIC solid18_reference_native)
set(engine_leaves s8ederic3 s8ejacip3 s8ederipr3 s8ederig3 s8eselecsh s8ederish2
  s8ea2bp8 s8eprst_ini s8edefo3 schkjabt3 srepiso3 sortho3 srrota3 sgcoor3
  sordeft3 s8edefot3 s8egetpij s8eselecsht)
set(engine_sources)
foreach(name IN LISTS engine_leaves)
  list(APPEND engine_sources "${prepared}/engine/${name}.F")
endforeach()
add_library(law90_solid18_current_native STATIC NativeEngineContext.F90 NativeGeometry.F
  NativeTotal.F NativeRate.F NativeCurrent.F ${engine_sources}
  "${prepared}/engine/constant_mod.F" "${prepared}/engine/precision_mod.F90"
  "${prepared}/engine/element_mod.F90")
target_link_libraries(law90_solid18_current_native PUBLIC law90_solid18_startup_native)
foreach(stage IN ITEMS starter engine)
  if(stage STREQUAL "starter")
    set(target law90_solid18_startup_native)
  else()
    set(target law90_solid18_current_native)
  endif()
  set(modules "${CMAKE_CURRENT_BINARY_DIR}/${stage}-modules")
  file(MAKE_DIRECTORY "${modules}")
  set_target_properties(${target} PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
  target_include_directories(${target} PRIVATE "${prepared}/${stage}" "${modules}")
  target_compile_definitions(${target} PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
  target_compile_options(${target} PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
    -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
  add_dependencies(${target} law90_solid18_verify_native)
endforeach()
add_test(NAME law90_solid18_native_identity
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --output "${prepared}" --check)
