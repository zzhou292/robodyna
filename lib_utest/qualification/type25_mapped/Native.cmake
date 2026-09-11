enable_language(Fortran)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(type25_native_dir "${CMAKE_CURRENT_LIST_DIR}/../type25/native")
add_library(type25_mapped_native STATIC native/Packet.F
  "${type25_native_dir}/NativeReference.F" "${type25_native_dir}/NativeFrame.F"
  "${type25_native_dir}/NativeDeformation.F" "${type25_native_dir}/NativeResponse.F"
  "${type25_native_dir}/NativeScatter.F" "${type25_native_dir}/NativeCoefficients.F")
target_include_directories(type25_mapped_native PRIVATE "${CMAKE_CURRENT_LIST_DIR}/native"
  "${type25_native_dir}" "${type25_native_dir}/stubs")
target_compile_options(type25_mapped_native PRIVATE -cpp -ffixed-line-length-none
  -fno-fast-math -ffp-contract=off -fcheck=all -finit-real=snan)
add_library(type25_mapped_native_adapter STATIC NativeValues.cpp ../type25/NativeOracle.cpp)
target_link_libraries(type25_mapped_native_adapter PUBLIC type25_mapped_values type25_mapped_native)
target_compile_options(type25_mapped_native_adapter PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(type25_mapped_native_test NativeTest.cpp)
target_link_libraries(type25_mapped_native_test PRIVATE type25_mapped_native_adapter GTest::gtest_main)
target_compile_options(type25_mapped_native_test PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME type25_mapped_native COMMAND type25_mapped_native_test)
add_test(NAME type25_mapped_source_identity COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_LIST_DIR}/native/verify_sources.py")
add_test(NAME type25_mapped_legacy_source_identity COMMAND "${Python3_EXECUTABLE}"
  "${type25_native_dir}/verify_sources.py")
set_tests_properties(type25_mapped_native PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
