include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../type13_recurrence/Native.cmake")
add_library(type13_resident_native STATIC
  "${CMAKE_CURRENT_LIST_DIR}/native/EndpointStiffness.F"
  "${CMAKE_CURRENT_LIST_DIR}/../type13_recurrence/NativeOracle.cpp")
target_link_libraries(type13_resident_native PUBLIC type13_h1_native tl_type13_math)
target_include_directories(type13_resident_native PRIVATE "${CMAKE_CURRENT_LIST_DIR}/native")
target_compile_options(type13_resident_native PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fno-fast-math;-ffp-contract=off;-fcheck=all;-finit-real=snan>")
