# Qualification-only extension of the already compiled private native contexts.
# Parent project supplies qeph_q1_native and t3_r3_native (unchanged old gates).
include_guard(GLOBAL)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
foreach(parent qeph_q1_native t3_r3_native)
  if(NOT TARGET ${parent})
    message(FATAL_ERROR "Global LAW1 native reference requires existing ${parent}")
  endif()
endforeach()
set(gl1_source "${CMAKE_CURRENT_LIST_DIR}")
set(gl1_generated "${CMAKE_CURRENT_BINARY_DIR}/global-law1-native-generated")
set(gl1_modules "${CMAKE_CURRENT_BINARY_DIR}/global-law1-native-modules")
file(MAKE_DIRECTORY "${gl1_modules}")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${gl1_source}/prepare.py" --output "${gl1_generated}"
  RESULT_VARIABLE gl1_status OUTPUT_VARIABLE gl1_receipt ERROR_VARIABLE gl1_error)
if(NOT gl1_status EQUAL 0)
  message(FATAL_ERROR "Global LAW1 native source gate failed: ${gl1_error}")
endif()
file(WRITE "${gl1_generated}/prepared-sources.json" "${gl1_receipt}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${gl1_source}/prepare.py" "${gl1_source}/source-manifest.json")
add_custom_target(shell_global_law1_native_sources
  COMMAND "${Python3_EXECUTABLE}" -B "${gl1_source}/prepare.py" --output "${gl1_generated}" --check VERBATIM)
function(global_law1_family target parent prefix)
  add_library(${target} STATIC "${gl1_generated}/${prefix}Material.F" "${gl1_generated}/${prefix}Law1.F"
    "${gl1_generated}/${prefix}Stiffness.F" "${gl1_generated}/${prefix}Force.F" "${gl1_source}/${prefix}Witness.F")
  foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(value ${parent} ${property})
    set_property(TARGET ${target} PROPERTY ${property} "${value}")
  endforeach()
  get_target_property(parent_modules ${parent} Fortran_MODULE_DIRECTORY)
  target_include_directories(${target} PRIVATE "${parent_modules}" "${gl1_modules}")
  set_target_properties(${target} PROPERTIES Fortran_MODULE_DIRECTORY "${gl1_modules}")
  add_dependencies(${target} ${parent} shell_global_law1_native_sources)
  target_link_libraries(${target} PUBLIC ${parent})
endfunction()
global_law1_family(shell_global_law1_native_qeph qeph_q1_native Qeph)
global_law1_family(shell_global_law1_native_t3 t3_r3_native T3)
add_library(shell_global_law1_native_reference STATIC "${gl1_generated}/QephAdapter.cpp"
  "${gl1_generated}/T3Adapter.cpp" "${gl1_source}/Witness.cpp")
target_include_directories(shell_global_law1_native_reference PUBLIC "${gl1_source}")
target_compile_features(shell_global_law1_native_reference PUBLIC cxx_std_17)
target_compile_options(shell_global_law1_native_reference PRIVATE -fno-fast-math -ffp-contract=off)
target_link_libraries(shell_global_law1_native_reference PUBLIC shell_global_law1_native_qeph shell_global_law1_native_t3)
add_dependencies(shell_global_law1_native_reference shell_global_law1_native_sources)
# The owning qualification already opted in with enable_testing().
add_test(NAME shell_global_law1_native_sources COMMAND "${Python3_EXECUTABLE}" -B "${gl1_source}/prepare.py"
  --output "${gl1_generated}" --check)
set_tests_properties(shell_global_law1_native_sources PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1)
