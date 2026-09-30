cmake_minimum_required(VERSION 3.22)
project(TLPhysicalOwnerForecast LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
get_filename_component(tl_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${tl_root}/lib_src/constraints/NodalRigidAssemblyBinding.cmake")
include("${tl_root}/lib_src/constraints/tied_shell/TiedCinAttachment.cmake")
add_executable(physical_owner_forecast_host_test HostTest.cpp ../rigid_assembly_owner/Fixture.cpp
  "${tl_root}/lib_src/solvers/NodalAssemblyCinForecast.cpp"
  "${tl_root}/lib_src/solvers/NodalRigidAssemblyStorage.cpp"
  "${tl_root}/lib_src/solvers/NodalCinStartup.cpp"
  "${tl_root}/lib_src/solvers/NodalCinState.cpp"
  "${tl_root}/lib_src/solvers/NodalRigidAssemblyChecks.cpp")
target_link_libraries(physical_owner_forecast_host_test PRIVATE tl_nodal_rigid_assembly_binding
  tl_tied_cin_attachment CUDA::cudart GTest::gtest_main)
target_compile_options(physical_owner_forecast_host_test PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME physical_owner_forecast_host COMMAND physical_owner_forecast_host_test)
option(PHYSICAL_OWNER_FORECAST_CUDA "Check exact actual combined owner allocation and retry" OFF)
if(PHYSICAL_OWNER_FORECAST_CUDA)
  enable_language(CUDA)
  add_subdirectory("${tl_root}/lib_src/solvers" "${CMAKE_CURRENT_BINARY_DIR}/owner")
  add_executable(physical_owner_forecast_cuda_test CudaTest.cu ../rigid_assembly_owner/Fixture.cpp)
  target_link_libraries(physical_owner_forecast_cuda_test PRIVATE tl_explicit_nodal_state GTest::gtest_main)
  set_target_properties(physical_owner_forecast_cuda_test PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(physical_owner_forecast_cuda_test PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
  add_test(NAME physical_owner_forecast_cuda COMMAND physical_owner_forecast_cuda_test)
endif()
