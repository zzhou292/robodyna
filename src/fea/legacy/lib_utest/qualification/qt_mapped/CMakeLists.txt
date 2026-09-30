cmake_minimum_required(VERSION 3.22)
project(TLQtMappedQualification LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
get_filename_component(tl_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
add_executable(qt_mapped_stiffness_host_test StiffnessTest.cpp)
target_include_directories(qt_mapped_stiffness_host_test PRIVATE "${tl_root}")
target_link_libraries(qt_mapped_stiffness_host_test PRIVATE GTest::gtest_main)
target_compile_features(qt_mapped_stiffness_host_test PRIVATE cxx_std_17)
target_compile_options(qt_mapped_stiffness_host_test PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME qt_mapped_stiffness_host COMMAND qt_mapped_stiffness_host_test)
set_tests_properties(qt_mapped_stiffness_host PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)

include("${tl_root}/lib_src/elements/ShellMappedStartup.cmake")
include("${tl_root}/lib_src/constraints/tied_shell/TiedCinAttachment.cmake")
add_executable(qt_mapped_startup_host_test StartupTest.cpp ResultTest.cpp OwnerFixture.cpp SkinFixtureTest.cpp
  ../rigid_assembly_owner/Fixture.cpp
  "${tl_root}/lib_src/elements/qeph/mapped/Startup.cpp"
  "${tl_root}/lib_src/elements/t3/mapped/Startup.cpp")
target_link_libraries(qt_mapped_startup_host_test PRIVATE tl_shell_mapped_startup
  tl_tied_cin_attachment GTest::gtest_main)
target_compile_options(qt_mapped_startup_host_test PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME qt_mapped_startup_host COMMAND qt_mapped_startup_host_test)
set_tests_properties(qt_mapped_startup_host PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)

include("${tl_root}/lib_src/elements/qbat/QbatBatchValues.cmake")
add_executable(qt_mapped_qbat_regression_test ../qbat_mapped/HostTest.cpp
  ../rigid_assembly_owner/Fixture.cpp)
target_link_libraries(qt_mapped_qbat_regression_test PRIVATE tl_qbat_batch_values
  tl_tied_cin_attachment GTest::gtest_main)
target_compile_options(qt_mapped_qbat_regression_test PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME qt_mapped_qbat_regression COMMAND qt_mapped_qbat_regression_test)
set_tests_properties(qt_mapped_qbat_regression PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)

option(QT_MAPPED_NATIVE "Independent complete native stiffness/scatter gates" OFF)
if(QT_MAPPED_NATIVE)
  include("${CMAKE_CURRENT_LIST_DIR}/Native.cmake")
endif()

option(QT_MAPPED_CUDA "Mapped QEPH/T3, CIN and explicit PART skin device gates" OFF)
if(QT_MAPPED_CUDA)
  include("${CMAKE_CURRENT_LIST_DIR}/Cuda.cmake")
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME qt_mapped_source_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
set_tests_properties(qt_mapped_source_identity PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)
