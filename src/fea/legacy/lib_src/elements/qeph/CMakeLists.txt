# Fixed-size host/device operations. Only qualification targets link the native oracle.
if(NOT TARGET tl_qeph)
  add_library(tl_qeph INTERFACE)
  target_include_directories(tl_qeph INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
  target_compile_features(tl_qeph INTERFACE cxx_std_17)
endif()
