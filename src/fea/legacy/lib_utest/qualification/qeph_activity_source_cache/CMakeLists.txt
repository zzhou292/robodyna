cmake_minimum_required(VERSION 3.22)
project(TLQephActivitySourceCache LANGUAGES CXX)
enable_testing()
set(QEPH_ACTIVITY_CUDA ON CACHE BOOL "Reuse actual mapped activity CUDA qualification" FORCE)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../qeph_mapped_activity" mapped-activity)
target_sources(qeph_mapped_activity_cuda PRIVATE "${CMAKE_CURRENT_LIST_DIR}/CertificateTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../rigid_assembly_owner/Fixture.cpp")
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME qeph_activity_source_cache_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
set_tests_properties(qeph_activity_source_cache_identity PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)
