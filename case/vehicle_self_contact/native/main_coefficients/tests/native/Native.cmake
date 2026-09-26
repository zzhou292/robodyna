include_guard(GLOBAL)
enable_language(C Fortran)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(_support_generated "${CMAKE_CURRENT_BINARY_DIR}/main-support-native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/prepare.py"
  --tl-root "${ROBO_DYNA_TL_ROOT}" --output "${_support_generated}" COMMAND_ERROR_IS_FATAL ANY)
file(GLOB _support_donors CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/original/*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_support_donors}
  "${CMAKE_CURRENT_LIST_DIR}/prepare.py" "${CMAKE_CURRENT_LIST_DIR}/source-manifest.json"
  "${CMAKE_CURRENT_LIST_DIR}/Support.F.in" "${CMAKE_CURRENT_LIST_DIR}/Order.F.in")
add_library(selected_main_support_native STATIC "${CMAKE_CURRENT_LIST_DIR}/../NativeOracle.cpp"
  "${_support_generated}/Constants.F90" "${_support_generated}/Element.F90"
  "${_support_generated}/Incoq.F" "${_support_generated}/Support.F" "${_support_generated}/Order.F"
  "${_support_generated}/my_orders.c")
set_target_properties(selected_main_support_native PROPERTIES Fortran_MODULE_DIRECTORY "${_support_generated}")
target_include_directories(selected_main_support_native PRIVATE "${_support_generated}")
target_link_libraries(selected_main_support_native PUBLIC robo_dyna_selected_shell_main_source)
target_compile_options(selected_main_support_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
add_test(NAME selected_main_support_native_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/prepare.py" --tl-root "${ROBO_DYNA_TL_ROOT}"
  --output "${_support_generated}" --check)
