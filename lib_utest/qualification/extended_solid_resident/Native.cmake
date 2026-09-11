# Root-only complete existing native donors; no copied production formulas.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_language(C)
set(REAR18_STARTUP_NATIVE ON)
set(REAR18_STARTUP_CUDA OFF)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../solid18_law44_startup" rear-startup)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../law90_preparation/native" preparation_native)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../law90_point/native" point_native)
# solid18_reference_native already belongs to the complete rear startup target.
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../law90_solid18_reference/native" kinematics_native)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../law90_solid18_force/native" foam_native)
add_library(extended_resident_native_checks STATIC "${CMAKE_CURRENT_LIST_DIR}/NativeChecks.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ConstructorChecks.cpp")
target_link_libraries(extended_resident_native_checks PUBLIC tl_solid_batch_values
  rear18_startup_oracle law90_solid18_force_native GTest::gtest)
target_compile_options(extended_resident_native_checks PRIVATE -fno-fast-math -ffp-contract=off)
