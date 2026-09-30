# Host/device value operations; only qualification targets link native Fortran.
if(NOT TARGET tl_t3)
  add_library(tl_t3 INTERFACE)
  target_include_directories(tl_t3 INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
  target_compile_features(tl_t3 INTERFACE cxx_std_17)
endif()
