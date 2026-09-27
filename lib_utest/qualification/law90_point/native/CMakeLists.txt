if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "LAW90 point qualifies GNU Fortran")
endif()
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/modules")
set(observer_args)
if(TL_LAW90_NATIVE_MODULUS_OBSERVER)
  set(observer_args --observe-modulus)
endif()
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" ${observer_args} RESULT_VARIABLE result)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "Complete LAW90 point donor authentication failed")
endif()
file(MAKE_DIRECTORY "${modules}")
add_library(law90_point_native STATIC NativePoint.F NativeContext.F90
  "${prepared}/sigeps90.F" "${prepared}/VALPVEC_V.F" "${prepared}/VALPVECDP_V.F"
  "${prepared}/precision.c" "${prepared}/vinter.F"
  "${prepared}/constant_mod.F" "${prepared}/precision_mod.F90")
set_target_properties(law90_point_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_include_directories(law90_point_native PRIVATE "${prepared}" "${modules}")
target_compile_definitions(law90_point_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(law90_point_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace>"
  -fno-fast-math -ffp-contract=off)
add_test(NAME law90_point_native_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --output "${prepared}" --check ${observer_args})

if(TL_LAW90_NATIVE_MODULUS_OBSERVER)
  target_sources(law90_point_native PRIVATE ModulusObserver.F90)
endif()
