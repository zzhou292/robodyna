cmake_minimum_required(VERSION 3.22)
project(TLCinMasterGather LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(tl_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(packet_dir "${CMAKE_CURRENT_LIST_DIR}/../cin_parallel_ordinary")
include("${tl_root}/lib_src/constraints/tied_shell/TiedCinAttachment.cmake")
add_executable(cin_master_gather_host HostTest.cpp CapacityTest.cpp
  "${packet_dir}/Packet.cpp" "${packet_dir}/Compare.cpp")
target_include_directories(cin_master_gather_host PRIVATE "${tl_root}" "${tl_root}/lib_src/solvers")
target_link_libraries(cin_master_gather_host PRIVATE tl_tied_cin_attachment GTest::gtest_main CUDA::toolkit)
target_compile_features(cin_master_gather_host PRIVATE cxx_std_17)
target_compile_options(cin_master_gather_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME cin_master_gather_host COMMAND cin_master_gather_host)
set_tests_properties(cin_master_gather_host PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
add_test(NAME cin_master_gather_sources
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
set_tests_properties(cin_master_gather_sources PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
add_test(NAME cin_master_gather_abi
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/abi_check.py")
set_tests_properties(cin_master_gather_abi PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)

option(CIN_MASTER_GATHER_CUDA "Frozen complete caller and actual owner CUDA qualification" OFF)
if(CIN_MASTER_GATHER_CUDA)
  enable_language(CUDA)
  add_subdirectory("${tl_root}/lib_src/solvers" nodal-owner)
  set(frozen "${CMAKE_CURRENT_BINARY_DIR}/Frozen.cu")
  add_custom_command(OUTPUT "${frozen}"
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/prepare_reference.py" "${frozen}"
    DEPENDS prepare_reference.py reference/ExplicitNodalCinStep.cu VERBATIM)
  add_executable(cin_master_gather_cuda CallerTest.cu OwnerTest.cu "${frozen}"
    "${packet_dir}/DevicePacket.cu" "${packet_dir}/Serial.cu"
    "${packet_dir}/Packet.cpp" "${packet_dir}/Compare.cpp"
    ../rigid_assembly_owner/Fixture.cpp)
  target_include_directories(cin_master_gather_cuda PRIVATE "${tl_root}" "${tl_root}/lib_src/solvers")
  target_link_libraries(cin_master_gather_cuda PRIVATE tl_explicit_nodal_state GTest::gtest_main CUDA::cudart)
  set_target_properties(cin_master_gather_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON)
  target_compile_options(cin_master_gather_cuda PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>)
  add_test(NAME cin_master_gather_cuda COMMAND cin_master_gather_cuda)
  set_tests_properties(cin_master_gather_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
endif()
