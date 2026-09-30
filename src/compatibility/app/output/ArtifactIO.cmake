# Shared artifact primitives for production output and focused host startup.
include_guard(GLOBAL)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)
add_library(robo_dyna_artifact_io STATIC
  "${CMAKE_CURRENT_LIST_DIR}/ArtifactIO.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ArtifactInventory.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/MeshArchive.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/GeometryMeshArchive.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SurfaceBindingFields.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CsvLedgerSegments.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CsvLedgerWriter.cpp")
target_link_libraries(robo_dyna_artifact_io PUBLIC Chrono::Chrono_core PRIVATE OpenSSL::Crypto)
target_include_directories(robo_dyna_artifact_io PUBLIC "${CMAKE_CURRENT_LIST_DIR}/..")
target_compile_features(robo_dyna_artifact_io PUBLIC cxx_std_17)
