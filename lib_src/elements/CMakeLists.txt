# Header-only fixed-size element arithmetic; no Chrono runtime dependency.
if(NOT TARGET tl_reissner_shell)
  add_library(tl_reissner_shell INTERFACE)
  target_include_directories(tl_reissner_shell INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/../.." "${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_compile_features(tl_reissner_shell INTERFACE cxx_std_17)
endif()
