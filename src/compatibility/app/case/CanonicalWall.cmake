# Core-only immutable wall reader, reusable by cases and offline replay.
# Tests and selection of a particular wall asset belong to the caller.
if(NOT TARGET crash_canonical_wall)
  add_library(crash_canonical_wall STATIC "${CMAKE_CURRENT_LIST_DIR}/CanonicalWall.cpp")
  target_compile_features(crash_canonical_wall PUBLIC cxx_std_17)
  target_include_directories(crash_canonical_wall PUBLIC "${CMAKE_CURRENT_LIST_DIR}/..")
  target_link_libraries(crash_canonical_wall PRIVATE Chrono::Chrono_core)
  target_compile_options(crash_canonical_wall PRIVATE -fno-fast-math -ffp-contract=off)
endif()
