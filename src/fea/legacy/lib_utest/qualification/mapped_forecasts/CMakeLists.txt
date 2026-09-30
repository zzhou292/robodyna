cmake_minimum_required(VERSION 3.22)
project(TLMappedForecastQualification LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
get_filename_component(tl_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${tl_root}/lib_src/elements/ShellMappedStartup.cmake")
include("${tl_root}/lib_src/elements/qbat/QbatBatchValues.cmake")
include("${tl_root}/lib_src/constraints/tied_shell/TiedCinAttachment.cmake")
add_executable(mapped_forecasts_host_test HostTest.cpp
  ../rigid_assembly_owner/Fixture.cpp
  "${tl_root}/lib_src/elements/qeph/mapped/Startup.cpp"
  "${tl_root}/lib_src/elements/t3/mapped/Startup.cpp"
  "${tl_root}/lib_src/elements/type25/mapped/Startup.cpp"
  "${tl_root}/lib_src/elements/type25/Type25BatchArena.cpp"
  "${tl_root}/lib_src/elements/type25/Type25BatchIdentity.cpp"
  "${tl_root}/lib_src/elements/type25/Type25BatchStartup.cpp")
target_link_libraries(mapped_forecasts_host_test PRIVATE tl_shell_mapped_startup
  tl_qbat_batch_values tl_tied_cin_attachment GTest::gtest_main)
target_compile_options(mapped_forecasts_host_test PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME mapped_forecasts_host COMMAND mapped_forecasts_host_test)
set_tests_properties(mapped_forecasts_host PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
