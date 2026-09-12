cmake_minimum_required(VERSION 3.22)
project(TLMappedWallNodeStatus LANGUAGES CXX)
enable_testing()
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
option(MAPPED_WALL_NODE_STATUS_CUDA "Compare complete frozen wall callers" OFF)

add_executable(mapped_wall_node_status_host HostTest.cpp)
target_include_directories(mapped_wall_node_status_host PRIVATE "${TL_ROOT}" "${CUDAToolkit_INCLUDE_DIRS}")
target_compile_features(mapped_wall_node_status_host PRIVATE cxx_std_17)
target_compile_options(mapped_wall_node_status_host PRIVATE -fno-fast-math -ffp-contract=off)
target_link_libraries(mapped_wall_node_status_host PRIVATE GTest::gtest_main)
add_test(NAME mapped_wall_node_status_host COMMAND mapped_wall_node_status_host)

if(MAPPED_WALL_NODE_STATUS_CUDA)
  enable_language(CUDA)
  include("${TL_ROOT}/lib_src/collision/NodalWallContact.cmake")
  include("${TL_ROOT}/lib_src/collision/PreparedPlanarWallQuery.cmake")
  include("${TL_ROOT}/lib_src/elements/ShellBatchBinding.cmake")
  add_executable(mapped_wall_node_status_cuda EvaluationTest.cu ScatterTest.cu
    "${TL_ROOT}/lib_utest/qualification/mapped_wall_scatter/Fixture.cpp"
    "${TL_ROOT}/lib_utest/qualification/vehicle_wall_device/VehicleWallFixture.cpp"
    "${TL_ROOT}/lib_utest/qualification/vehicle_wall_device/VehicleWallChecks.cpp"
    "${TL_ROOT}/lib_src/collision/NodalWallContactArena.cpp"
    "${TL_ROOT}/lib_src/collision/NodalWallContactModel.cpp"
    "${TL_ROOT}/lib_src/collision/NodalWallContactIncidence.cpp")
  target_include_directories(mapped_wall_node_status_cuda PRIVATE "${TL_ROOT}" "${CUDAToolkit_INCLUDE_DIRS}")
  target_link_libraries(mapped_wall_node_status_cuda PRIVATE tl_nodal_wall_contact
    tl_prepared_planar_wall_query tl_shell_batch_binding CUDA::cudart GTest::gtest_main)
  set_target_properties(mapped_wall_node_status_cuda PROPERTIES CXX_STANDARD 17 CUDA_STANDARD 17)
  target_compile_options(mapped_wall_node_status_cuda PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off;-ffunction-sections;-fdata-sections>"
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off,-ffunction-sections,-fdata-sections>")
  target_link_options(mapped_wall_node_status_cuda PRIVATE "LINKER:--gc-sections")
  add_test(NAME mapped_wall_node_status_cuda COMMAND mapped_wall_node_status_cuda)
  set_tests_properties(mapped_wall_node_status_cuda PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1)
endif()

add_test(NAME mapped_wall_node_status_identity COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
set_tests_properties(mapped_wall_node_status_host mapped_wall_node_status_identity PROPERTIES
  TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1)
