# Independent bounded native source qualification only. No production target
# depends on this library or on any OpenRadioss executable/library.
include_guard(GLOBAL)
enable_language(C Fortran)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(_coated_native "${CMAKE_CURRENT_BINARY_DIR}/coated-native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/prepare.py"
  --tl-root "${ROBO_DYNA_TL_ROOT}" --output "${_coated_native}"
  RESULT_VARIABLE _coated_prepared ERROR_VARIABLE _coated_error)
if(NOT _coated_prepared EQUAL 0)
  message(FATAL_ERROR "Independent coated source preparation failed: ${_coated_error}")
endif()
file(GLOB _coated_donors CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/original/*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_coated_donors}
  "${CMAKE_CURRENT_LIST_DIR}/prepare.py" "${CMAKE_CURRENT_LIST_DIR}/source-manifest.json"
  "${CMAKE_CURRENT_LIST_DIR}/Reader.F.in" "${CMAKE_CURRENT_LIST_DIR}/Roles.F90"
  "${CMAKE_CURRENT_LIST_DIR}/Order.F.in"
  "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/radioss_type25_selection/native/Sources.py")
add_library(v5_coated_native_reference STATIC
  "${CMAKE_CURRENT_LIST_DIR}/../NativeOracle.cpp"
  "${_coated_native}/Constants.F90" "${_coated_native}/Element.F90"
  "${_coated_native}/Classification.F" "${_coated_native}/ChecksVolume.F"
  "${_coated_native}/Reader.F" "${_coated_native}/Roles.F90" "${_coated_native}/Order.F"
  "${_coated_native}/my_orders.c")
set_target_properties(v5_coated_native_reference PROPERTIES Fortran_MODULE_DIRECTORY "${_coated_native}")
target_include_directories(v5_coated_native_reference PRIVATE "${_coated_native}")
target_link_libraries(v5_coated_native_reference PUBLIC robo_dyna_v5_coated_source)
target_compile_features(v5_coated_native_reference PUBLIC cxx_std_17)
target_compile_options(v5_coated_native_reference PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
add_test(NAME v5_coated_native_reference_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/prepare.py" --tl-root "${ROBO_DYNA_TL_ROOT}"
  --output "${_coated_native}" --check)
set_tests_properties(v5_coated_native_reference_source PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
