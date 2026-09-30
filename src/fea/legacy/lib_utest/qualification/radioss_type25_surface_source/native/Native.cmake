include_guard(GLOBAL)
enable_language(C Fortran)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Pinned early-surface oracle requires explicit GNU Fortran")
endif()
get_filename_component(_surface_tl "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(_surface_generated "${CMAKE_CURRENT_BINARY_DIR}/surface-native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/prepare.py"
  --tl-root "${_surface_tl}" --output "${_surface_generated}" COMMAND_ERROR_IS_FATAL ANY)
file(GLOB _surface_donors CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/original/*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_surface_donors}
  "${CMAKE_CURRENT_LIST_DIR}/prepare.py" "${CMAKE_CURRENT_LIST_DIR}/source-manifest.json"
  "${CMAKE_CURRENT_LIST_DIR}/Modules.F90" "${CMAKE_CURRENT_LIST_DIR}/Wrapper.F.in"
  "${_surface_tl}/lib_utest/qualification/radioss_type25_selection/native/Sources.py")
add_library(type25_surface_source_native STATIC "${CMAKE_CURRENT_LIST_DIR}/../NativeOracle.cpp"
  "${_surface_generated}/Modules.F90" "${_surface_generated}/Wrapper.F"
  "${_surface_generated}/Parts.F" "${_surface_generated}/Create.F"
  "${_surface_generated}/Buffer.F" "${_surface_generated}/Solid.F"
  "${_surface_generated}/Shell.F" "${_surface_generated}/my_orders.c")
set_target_properties(type25_surface_source_native PROPERTIES Fortran_MODULE_DIRECTORY "${_surface_generated}")
target_include_directories(type25_surface_source_native PUBLIC "${_surface_tl}" PRIVATE "${_surface_generated}")
target_compile_features(type25_surface_source_native PUBLIC cxx_std_17)
target_compile_options(type25_surface_source_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
add_test(NAME type25_surface_source_native_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/prepare.py" --tl-root "${_surface_tl}" --output "${_surface_generated}" --check)
set_tests_properties(type25_surface_source_native_source PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
