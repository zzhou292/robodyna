include_guard(GLOBAL)
enable_language(Fortran)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Whole solid support source oracle requires GNU C++ and Fortran")
endif()
set(_solid_support_generated "${CMAKE_CURRENT_BINARY_DIR}/solid-support-native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/prepare.py"
  --tl-root "${ROBO_DYNA_TL_ROOT}" --output "${_solid_support_generated}" COMMAND_ERROR_IS_FATAL ANY)
file(GLOB _solid_support_donors CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/original/*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_solid_support_donors}
  "${CMAKE_CURRENT_LIST_DIR}/prepare.py" "${CMAKE_CURRENT_LIST_DIR}/source-manifest.json"
  "${CMAKE_CURRENT_LIST_DIR}/Boundary.F90" "${CMAKE_CURRENT_LIST_DIR}/Wrapper.F.in")
add_library(mixed_solid_support_native STATIC "${CMAKE_CURRENT_LIST_DIR}/../NativeSolidSupport.cpp"
  "${_solid_support_generated}/Boundary.F90" "${_solid_support_generated}/Constants.F90"
  "${_solid_support_generated}/Insol.F" "${_solid_support_generated}/Normal.F" "${_solid_support_generated}/Wrapper.F")
set_target_properties(mixed_solid_support_native PROPERTIES Fortran_MODULE_DIRECTORY "${_solid_support_generated}")
target_include_directories(mixed_solid_support_native PRIVATE "${_solid_support_generated}")
target_link_libraries(mixed_solid_support_native PUBLIC robo_dyna_selected_shell_main_source)
target_compile_options(mixed_solid_support_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
add_test(NAME mixed_solid_support_native_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/prepare.py" --tl-root "${ROBO_DYNA_TL_ROOT}"
  --output "${_solid_support_generated}" --check)
