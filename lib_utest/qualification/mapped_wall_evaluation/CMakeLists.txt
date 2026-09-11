cmake_minimum_required(VERSION 3.22)
project(TLMappedWallEvaluation LANGUAGES CXX CUDA)
enable_testing()
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
include("${TL_ROOT}/lib_src/collision/NodalWallContact.cmake")
include("${TL_ROOT}/lib_src/collision/PreparedPlanarWallQuery.cmake")
include("${TL_ROOT}/lib_src/elements/ShellBatchBinding.cmake")
add_executable(mapped_wall_evaluation_check CudaTest.cu
  "${TL_ROOT}/lib_utest/qualification/vehicle_wall_device/VehicleWallFixture.cpp"
  "${TL_ROOT}/lib_utest/qualification/vehicle_wall_device/VehicleWallChecks.cpp"
  "${TL_ROOT}/lib_src/collision/NodalWallContactArena.cpp"
  "${TL_ROOT}/lib_src/collision/NodalWallContactModel.cpp"
  "${TL_ROOT}/lib_src/collision/NodalWallContactIncidence.cpp")
target_include_directories(mapped_wall_evaluation_check PRIVATE "${TL_ROOT}" "${CUDAToolkit_INCLUDE_DIRS}")
target_link_libraries(mapped_wall_evaluation_check PRIVATE tl_nodal_wall_contact
  tl_prepared_planar_wall_query tl_shell_batch_binding CUDA::cudart GTest::gtest_main)
set_target_properties(mapped_wall_evaluation_check PROPERTIES CXX_STANDARD 17 CUDA_STANDARD 17)
target_compile_options(mapped_wall_evaluation_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off;-ffunction-sections;-fdata-sections>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off,-ffunction-sections,-fdata-sections>")
target_link_options(mapped_wall_evaluation_check PRIVATE "LINKER:--gc-sections")
add_test(NAME mapped_wall_evaluation_cuda COMMAND mapped_wall_evaluation_check)
set_tests_properties(mapped_wall_evaluation_cuda PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1)

# Freeze the independent oracle and unchanged arithmetic leaves before compile.
file(READ "${CMAKE_CURRENT_LIST_DIR}/source-manifest.json" receipt)
file(SHA256 "${CMAKE_CURRENT_LIST_DIR}/source-manifest.json" receipt_sha256)
if(NOT receipt_sha256 STREQUAL "25ff85470fb3253d7163858160089db61b487fcaf6e72471983cdda59c9f770c")
  message(FATAL_ERROR "Frozen wall source receipt differs")
endif()
string(JSON count LENGTH "${receipt}" files)
math(EXPR last "${count}-1")
foreach(index RANGE 0 ${last})
  string(JSON path GET "${receipt}" files ${index} path)
  string(JSON expected GET "${receipt}" files ${index} sha256)
  file(SHA256 "${TL_ROOT}/${path}" actual)
  if(NOT actual STREQUAL expected)
    message(FATAL_ERROR "Frozen wall reference or shared arithmetic changed: ${path}")
  endif()
endforeach()
