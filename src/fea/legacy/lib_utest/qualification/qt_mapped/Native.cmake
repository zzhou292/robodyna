enable_language(Fortran)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(TL_PLACEMENT_FORCE_NATIVE ON CACHE BOOL "" FORCE)
set(TL_PLACEMENT_FORCE_CUDA OFF CACHE BOOL "" FORCE)
add_subdirectory("../shell_placement_force" placed-native)
include("${CMAKE_CURRENT_LIST_DIR}/../native/law44_one_point/Law44OnePoint.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../t3_one_point/OnePointNative.cmake")
add_executable(qt_mapped_native_test NativeLayeredTest.cpp NativeOnePointTest.cpp
  ../t3_one_point/NativeOracle.cpp)
target_link_libraries(qt_mapped_native_test PRIVATE shell_placement_force_test_support
  t3_one_point_native GTest::gtest_main)
target_compile_options(qt_mapped_native_test PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME qt_mapped_native COMMAND qt_mapped_native_test)
set_tests_properties(qt_mapped_native PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
