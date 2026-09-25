cmake_minimum_required(VERSION 3.22)
project(TLRadiossType25FixedMainStartup LANGUAGES CXX Fortran)
enable_testing()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "This pinned startup oracle qualifies GNU C++ and Fortran")
endif()
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/collision/RadiossType25FixedMainStartup.cmake")
set(native "${CMAKE_CURRENT_BINARY_DIR}/native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/native/prepare.py"
  --output "${native}" RESULT_VARIABLE prepared ERROR_VARIABLE diagnostic)
if(NOT prepared EQUAL 0)
  message(FATAL_ERROR "Pinned startup preparation failed: ${diagnostic}")
endif()
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/observed.py"
  --output "${CMAKE_CURRENT_BINARY_DIR}/Observed.h" RESULT_VARIABLE prepared ERROR_VARIABLE diagnostic)
if(NOT prepared EQUAL 0)
  message(FATAL_ERROR "Observed startup evidence preparation failed: ${diagnostic}")
endif()
file(GLOB donors CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/native/original/*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${donors}
  native/prepare.py native/Sources.py native/Extraction.py native/source-manifest.json
  native/Memory.F90 native/Boundary.F90 native/NormalStorage.F90 native/Wrapper.F90 native/SelectorWrapper.F90 native/CsrWrapper.F.in
  ../radioss_type25_selection/native/Sources.py observed.py evidence/manifest.json evidence/native-observations.jsonl)
set(generated Constants.F90 Memory.F90 Boundary.F90 NormalStorage.F90 Expand.F Neighborhood.F
  I25NEIGH_SEG_EN.F I25NEIGH_SEG_E.F I25NEIGH_REMOVEALLBUT1.F I25NEIGH_SEG_OPP.F I25NEIGH_MSG_ERR.F
  ADD_ID.F INTAB.F SAME_SEG.F NORMV3.F NORMA4N.F StarterNormals.F ReadyNormals.F FreeMain.F CsrWrapper.F Wrapper.F90 SelectorWrapper.F90)
list(TRANSFORM generated PREPEND "${native}/")
add_library(type25_startup_oracle STATIC NativeOracle.cpp SelectorOracle.cpp ${generated})
set_target_properties(type25_startup_oracle PROPERTIES Fortran_MODULE_DIRECTORY "${native}")
target_include_directories(type25_startup_oracle PRIVATE "${native}")
target_link_libraries(type25_startup_oracle PUBLIC tl_radioss_type25_fixed_main_startup)
target_compile_options(type25_startup_oracle PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>")
add_executable(type25_fixed_startup_host HostTest.cpp ObservedTest.cpp GeneralTopologyTest.cpp CoatedStartupTest.cpp)
target_include_directories(type25_fixed_startup_host PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_link_libraries(type25_fixed_startup_host PRIVATE type25_startup_oracle GTest::gtest_main)
add_test(NAME type25_fixed_startup_host COMMAND type25_fixed_startup_host)
add_executable(type25_fixed_startup_consumer Consumer.cpp)
# No private precision flags, native reference, GTest or CUDA in this consumer.
target_link_libraries(type25_fixed_startup_consumer PRIVATE tl_radioss_type25_fixed_main_startup)
add_test(NAME type25_fixed_startup_consumer COMMAND type25_fixed_startup_consumer)
add_test(NAME type25_fixed_startup_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/native/prepare.py" --output "${native}" --check)
add_test(NAME type25_fixed_startup_observed_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/observed.py" --output "${CMAKE_CURRENT_BINARY_DIR}/Observed.h" --check)
set_tests_properties(type25_fixed_startup_host type25_fixed_startup_consumer
  type25_fixed_startup_source type25_fixed_startup_observed_source
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
