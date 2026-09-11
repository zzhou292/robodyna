# Pure host startup/layout/identity; no device allocation or native solver.
include_guard(GLOBAL)
find_package(CUDAToolkit REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/QbatForce.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../ShellFormulationScope.cmake")
add_library(tl_qbat_batch_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/QbatBatchArena.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/QbatBatchStartup.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/QbatBatchIdentity.cpp")
target_link_libraries(tl_qbat_batch_values PUBLIC tl_qbat_force tl_shell_formulation_scope)
target_include_directories(tl_qbat_batch_values PUBLIC "${CUDAToolkit_INCLUDE_DIRS}")
target_compile_features(tl_qbat_batch_values PUBLIC cxx_std_17)
target_compile_options(tl_qbat_batch_values PRIVATE -fno-fast-math -ffp-contract=off)
