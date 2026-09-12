cmake_minimum_required(VERSION 3.22)
project(TLShellAffectedQualification LANGUAGES CXX CUDA Fortran)
enable_testing()

# Imported dependencies must be visible to sibling qualification directories.
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

# This entrypoint owns the complete four-project gate. Standalone defaults stay
# in the child projects; normal variables also override stale cache toggles here.
set(TL_RESIDENT_SHELL_FAILURE_CUDA ON)
set(QT_MAPPED_NATIVE ON)
set(QT_MAPPED_CUDA ON)
set(QEPH_GATHER_CUDA ON)
set(QEPH_Q1_BUILD_TESTS ON)
set(T3_R1_BUILD_TESTS ON)
set(T3_R2_BUILD_TESTS ON)
set(T3_R3_BUILD_TESTS ON)

# Resident failure creates the full native T3 R1/R2/R3 and QEPH owners first.
# Later projects reuse those targets and their private prepared/module contexts.
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../resident_shell_failure" resident-shell-failure)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../qt_mapped" qt-mapped)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../shell_parent_activity" shell-parent-activity)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../qeph_mapped_gather" qeph-mapped-gather)

# Require the original union without filtering new tests or merging executables.
function(shell_affected_collect_tests directory output)
  get_property(tests DIRECTORY "${directory}" PROPERTY TESTS)
  get_property(children DIRECTORY "${directory}" PROPERTY SUBDIRECTORIES)
  foreach(child IN LISTS children)
    shell_affected_collect_tests("${child}" child_tests)
    list(APPEND tests ${child_tests})
  endforeach()
  set(${output} "${tests}" PARENT_SCOPE)
endfunction()
shell_affected_collect_tests("${CMAKE_CURRENT_SOURCE_DIR}" shell_affected_tests)
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/ExpectedTests.txt" shell_affected_required)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/ExpectedTests.txt")
foreach(required IN LISTS shell_affected_required)
  if(NOT required IN_LIST shell_affected_tests)
    message(FATAL_ERROR "Combined shell qualification omitted required test: ${required}")
  endif()
endforeach()
list(LENGTH shell_affected_tests shell_affected_count)
list(REMOVE_DUPLICATES shell_affected_tests)
list(LENGTH shell_affected_tests shell_affected_distinct)
if(NOT shell_affected_count EQUAL shell_affected_distinct)
  message(FATAL_ERROR "Combined shell qualification has duplicate CTest names")
endif()
message(STATUS "Combined shell qualification retains ${shell_affected_distinct} distinct tests")
