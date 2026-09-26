include_guard(GLOBAL)
enable_language(Fortran)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "The independent initial-history oracle requires the pinned GNU Fortran compiler")
endif()
set(initial_native_dir "${CMAKE_CURRENT_BINARY_DIR}/initial-state-native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/prepare.py"
  --tl-root "${TYPE25_INITIAL_ROOT}" --output "${initial_native_dir}" RESULT_VARIABLE initial_native_result)
if(NOT initial_native_result EQUAL 0)
  message(FATAL_ERROR "Independent initial-history source preparation failed")
endif()
file(GLOB initial_donors CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/original/*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${initial_donors}
  "${CMAKE_CURRENT_LIST_DIR}/prepare.py" "${CMAKE_CURRENT_LIST_DIR}/source-manifest.json"
  "${CMAKE_CURRENT_LIST_DIR}/Wrapper.F90" "${CMAKE_CURRENT_LIST_DIR}/Boundary.F90")
set(initial_generated Constants.F90 Boundary.F90 I25COR3.F I25PEN3.F I25PWR3.F Wrapper.F90)
list(TRANSFORM initial_generated PREPEND "${initial_native_dir}/")
add_library(type25_initial_state_native STATIC "${CMAKE_CURRENT_LIST_DIR}/../NativeOracle.cpp" ${initial_generated})
target_compile_features(type25_initial_state_native PUBLIC cxx_std_17)
set_target_properties(type25_initial_state_native PROPERTIES Fortran_MODULE_DIRECTORY "${initial_native_dir}")
target_include_directories(type25_initial_state_native PRIVATE "${initial_native_dir}" PUBLIC "${TYPE25_INITIAL_ROOT}")
target_compile_options(type25_initial_state_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
add_test(NAME type25_initial_state_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/prepare.py" --tl-root "${TYPE25_INITIAL_ROOT}" --output "${initial_native_dir}" --check)
