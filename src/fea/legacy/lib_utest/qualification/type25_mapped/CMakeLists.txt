cmake_minimum_required(VERSION 3.22)
project(TLType25MappedQualification LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
get_filename_component(tl_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${tl_root}/lib_src/elements/ShellPhysicalOwnerValues.cmake")
include("${tl_root}/lib_src/assembly/NodalMassBinding.cmake")
include("${tl_root}/lib_src/constraints/tied_shell/TiedCinAttachment.cmake")
add_library(type25_mapped_values STATIC
  "${tl_root}/lib_src/elements/type25/Type25BatchArena.cpp"
  "${tl_root}/lib_src/elements/type25/Type25BatchStartup.cpp"
  "${tl_root}/lib_src/elements/type25/Type25BatchIdentity.cpp"
  "${tl_root}/lib_src/elements/type25/mapped/Startup.cpp")
target_link_libraries(type25_mapped_values PUBLIC tl_shell_physical_owner_values tl_nodal_mass_binding)
target_compile_options(type25_mapped_values PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(type25_mapped_host_test HostTest.cpp ../rigid_assembly_owner/Fixture.cpp
  ../type25_batch/LayoutTest.cpp ../type25_batch/VehicleLayoutTest.cpp)
target_link_libraries(type25_mapped_host_test PRIVATE type25_mapped_values tl_tied_cin_attachment GTest::gtest_main)
target_compile_options(type25_mapped_host_test PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME type25_mapped_host COMMAND type25_mapped_host_test)
set_tests_properties(type25_mapped_host PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
option(TYPE25_MAPPED_NATIVE "Independent native nodal stiffness and interval tests" OFF)
option(TYPE25_MAPPED_CUDA "Actual mapped physical owner and rollback checks" OFF)
if(TYPE25_MAPPED_NATIVE)
  include("${CMAKE_CURRENT_LIST_DIR}/Native.cmake")
endif()
if(TYPE25_MAPPED_CUDA)
  include("${CMAKE_CURRENT_LIST_DIR}/Cuda.cmake")
endif()
