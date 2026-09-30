include_guard(GLOBAL)
if(NOT TARGET type25_startup_oracle OR NOT TARGET type25_surface_source_native)
  message(FATAL_ERROR "Mixed interface reference reuses both complete owning native libraries")
endif()
get_filename_component(_interface_tl "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(_interface_generated "${CMAKE_CURRENT_BINARY_DIR}/interface-native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/prepare.py"
  --tl-root "${_interface_tl}" --output "${_interface_generated}" COMMAND_ERROR_IS_FATAL ANY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/prepare.py" "${CMAKE_CURRENT_LIST_DIR}/source-manifest.json"
  "${CMAKE_CURRENT_LIST_DIR}/Wrapper.F.in" "${CMAKE_CURRENT_LIST_DIR}/Observation.F90"
  "${CMAKE_CURRENT_LIST_DIR}/original/i24surfi.F"
  "${_interface_tl}/lib_utest/qualification/radioss_type25_fixed_main_startup/native/original/i25surfi.F"
  "${_interface_tl}/lib_utest/qualification/radioss_type25_surface_source/native/original/build_cnel.F"
  "${_interface_tl}/lib_utest/qualification/radioss_type25_selection/native/Sources.py")
get_target_property(_startup_modules type25_startup_oracle Fortran_MODULE_DIRECTORY)
target_sources(type25_surface_source_native PRIVATE
  "${_interface_generated}/Observation.F90" "${_interface_generated}/Classification.F"
  "${_interface_generated}/InterfaceWrapper.F")
target_include_directories(type25_surface_source_native PRIVATE "${_interface_generated}" "${_startup_modules}")
target_link_libraries(type25_surface_source_native PRIVATE type25_startup_oracle)
add_library(type25_interface_oracle STATIC "${CMAKE_CURRENT_LIST_DIR}/../NativeOracle.cpp")
target_link_libraries(type25_interface_oracle PUBLIC type25_surface_source_native tl_radioss_type25_interface_surface)
target_compile_features(type25_interface_oracle PUBLIC cxx_std_17)
target_compile_options(type25_interface_oracle PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME type25_interface_native_source COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/prepare.py"
  --tl-root "${_interface_tl}" --output "${_interface_generated}" --check)
set_tests_properties(type25_interface_native_source PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
