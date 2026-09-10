cmake_minimum_required(VERSION 3.22)
project(TLResidentPlasticityQualification LANGUAGES CXX CUDA Fortran)
# Standalone owner for the existing resident gate; all mechanics/native targets
# are supplied by their established owning CMake modules.
enable_testing()
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
set(TL_T3_ENABLE_CUDA ON)
set(TL_T3_ENABLE_FORCE ON)
set(TL_T3_ENABLE_BATCH ON)
set(TL_T3_ENABLE_MIXED_BINDING ON)
set(TL_T3_ENABLE_MIXED ON)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../t3" t3)
include("${CMAKE_CURRENT_SOURCE_DIR}/ResidentPlasticity.cmake")
