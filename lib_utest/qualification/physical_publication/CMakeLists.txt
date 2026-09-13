cmake_minimum_required(VERSION 3.22)
project(TLPhysicalPublication LANGUAGES CXX C)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
option(PHYSICAL_PUBLICATION_CUDA "Complete physical owner transaction gate" OFF)
if(PHYSICAL_PUBLICATION_CUDA)
  enable_language(CUDA)
  add_subdirectory("${TL_ROOT}/lib_src/solvers" nodal-owner)
  include("${TL_ROOT}/lib_src/elements/qeph/QephBatch.cmake")
  include("${TL_ROOT}/lib_src/elements/t3/T3Batch.cmake")
  include("${TL_ROOT}/lib_src/elements/ShellBatchPublication.cmake")
else()
  include("${TL_ROOT}/lib_src/elements/type45/resident/Values.cmake")
  include("${TL_ROOT}/lib_src/elements/qbat/QbatBatchValues.cmake")
  include("${TL_ROOT}/lib_src/elements/type13/resident/Values.cmake")
  add_subdirectory("${TL_ROOT}/lib_src/elements/solids/resident" solid-values)
  add_subdirectory("${TL_ROOT}/lib_src/elements/beam18/resident" beam18-values)
endif()
include("${TL_ROOT}/lib_src/constraints/tied_shell/TiedCinAttachment.cmake")
include("${TL_ROOT}/lib_src/constraints/NodalRigidAssemblyBinding.cmake")
add_library(physical_publication_fixture STATIC Sources.cpp Constraints.cpp)
target_include_directories(physical_publication_fixture PUBLIC "${TL_ROOT}")
target_link_libraries(physical_publication_fixture PUBLIC tl_solid_batch_values
  tl_shell_physical_owner_values tl_nodal_rigid_assembly_binding tl_tied_cin_attachment GTest::gtest_main)
target_compile_features(physical_publication_fixture PUBLIC cxx_std_17)
target_compile_options(physical_publication_fixture PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(physical_publication_host HostTest.cpp Type13MappedHostTest.cpp)
if(PHYSICAL_PUBLICATION_CUDA)
  target_link_libraries(physical_publication_host PRIVATE physical_publication_fixture tl_shell_batch_publication)
  include("${CMAKE_CURRENT_LIST_DIR}/Cuda.cmake")
else()
  # Discard unused CUDA-owner legacy helper functions in this host value gate.
  target_sources(physical_publication_host PRIVATE
    "${TL_ROOT}/lib_src/elements/publication/PhysicalValues.cpp"
    "${TL_ROOT}/lib_src/elements/publication/PhysicalForecast.cpp"
    "${TL_ROOT}/lib_src/elements/publication/ShellPhysicalScratchParticipation.cpp"
    "${TL_ROOT}/lib_src/elements/ShellBatchPublicationValues.cpp"
    "${TL_ROOT}/lib_src/elements/qeph/QephBatchIdentity.cpp"
    "${TL_ROOT}/lib_src/elements/t3/T3BatchIdentity.cpp"
    "${TL_ROOT}/lib_src/elements/type25/Type25BatchIdentity.cpp")
  target_compile_options(physical_publication_host PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(physical_publication_host PRIVATE -Wl,--gc-sections)
  target_link_libraries(physical_publication_host PRIVATE physical_publication_fixture
    tl_qbat_batch_values tl_type13_batch_values tl_type45_batch_values tl_beam18_batch_values)
endif()
target_compile_options(physical_publication_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME physical_publication_host COMMAND physical_publication_host)
set_tests_properties(physical_publication_host PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
add_test(NAME physical_scratch_participation_source
  COMMAND "${Python3_EXECUTABLE}" -B
          "${CMAKE_CURRENT_LIST_DIR}/verify_participation_sources.py")
add_test(NAME physical_scratch_participation_syntax
  COMMAND "${Python3_EXECUTABLE}" -B
          "${CMAKE_CURRENT_LIST_DIR}/check_participation_syntax.py"
          --compiler "${CMAKE_CXX_COMPILER}"
          --output "${CMAKE_CURRENT_BINARY_DIR}/participation-syntax")
set_tests_properties(physical_scratch_participation_source
  physical_scratch_participation_syntax
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)

add_executable(type13_mapped_legacy_host "${CMAKE_CURRENT_LIST_DIR}/../type13_resident/HostTest.cpp")
target_link_libraries(type13_mapped_legacy_host PRIVATE tl_type13_batch_values GTest::gtest_main)
target_compile_options(type13_mapped_legacy_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME type13_mapped_legacy_host COMMAND type13_mapped_legacy_host)
