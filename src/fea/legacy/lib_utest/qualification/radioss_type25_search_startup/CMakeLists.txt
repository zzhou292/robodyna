cmake_minimum_required(VERSION 3.22)
project(TLRadiossType25SearchStartup LANGUAGES CXX Fortran)
enable_testing()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "The pinned serial native oracle qualifies GNU C++ and Fortran")
endif()
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/collision/RadiossType25SearchStartup.cmake")
set(native "${CMAKE_CURRENT_BINARY_DIR}/native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/native/prepare.py"
  --output "${native}" RESULT_VARIABLE code ERROR_VARIABLE error)
if(NOT code EQUAL 0)
  message(FATAL_ERROR "Native startup source preparation failed: ${error}")
endif()
foreach(which IN ITEMS Mesh Search)
  if(which STREQUAL "Mesh")
    set(script "${CMAKE_CURRENT_LIST_DIR}/../radioss_type25_fixed_main_startup/observed.py")
  else()
    set(script "${CMAKE_CURRENT_LIST_DIR}/observed.py")
  endif()
  execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${script}" --output "${CMAKE_CURRENT_BINARY_DIR}/Observed${which}.h"
    RESULT_VARIABLE code ERROR_VARIABLE error)
  if(NOT code EQUAL 0)
    message(FATAL_ERROR "Pinned observation preparation failed: ${error}")
  endif()
  add_test(NAME type25_search_startup_observed_${which} COMMAND "${Python3_EXECUTABLE}" -B "${script}"
    --output "${CMAKE_CURRENT_BINARY_DIR}/Observed${which}.h" --check)
endforeach()
file(GLOB donors CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/native/original/*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${donors}
  native/prepare.py native/Sources.py native/source-manifest.json native/Boundary.F90 native/Wrapper.F90
  native/Margin.F.in native/Extent.F90.in native/InitialFlags.F.in observed.py
  ../radioss_type25_selection/native/Sources.py ../radioss_type25_fixed_main_startup/observed.py
  ../radioss_type25_fixed_main_startup/evidence/manifest.json ../radioss_type25_fixed_main_startup/evidence/native-observations.jsonl)
set(generated Constants.F90 Boundary.F90 Removal.F90 SmallMargin.F90 RemovalInit.F Inverse.F
  Margin.F Extent.F90 InitialFlags.F Wrapper.F90)
list(TRANSFORM generated PREPEND "${native}/")
add_library(type25_search_startup_oracle STATIC NativeOracle.cpp ${generated})
set_target_properties(type25_search_startup_oracle PROPERTIES Fortran_MODULE_DIRECTORY "${native}")
target_include_directories(type25_search_startup_oracle PRIVATE "${native}")
target_link_libraries(type25_search_startup_oracle PUBLIC tl_radioss_type25_search_startup)
# No OpenMP flag: this reference deliberately owns serial native preprocessing.
target_compile_options(type25_search_startup_oracle PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>")
add_executable(type25_search_startup_host HostTest.cpp ObservedTest.cpp ContextTest.cpp)
target_include_directories(type25_search_startup_host PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_link_libraries(type25_search_startup_host PRIVATE type25_search_startup_oracle GTest::gtest_main)
add_test(NAME type25_search_startup_host COMMAND type25_search_startup_host)
add_executable(type25_search_startup_consumer Consumer.cpp)
target_link_libraries(type25_search_startup_consumer PRIVATE tl_radioss_type25_search_startup)
add_test(NAME type25_search_startup_consumer COMMAND type25_search_startup_consumer)
add_test(NAME type25_search_startup_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/native/prepare.py" --output "${native}" --check)
get_property(alltests DIRECTORY PROPERTY TESTS)
set_tests_properties(${alltests} PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
