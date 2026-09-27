include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../assembly/SolidNodeContributions.cmake")
find_package(Eigen3 REQUIRED)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/control" "${CMAKE_CURRENT_BINARY_DIR}/control")
add_library(tl_solid_model STATIC Model.cpp model/Budget.cpp model/Materials.cpp
  model/Parents.cpp model/Identity.cpp model/Readback.cpp model/ExtendedMaterials.cpp model/ExtendedIdentity.cpp)
target_link_libraries(tl_solid_model PUBLIC tl_solid_node_contributions tl_solid_control_selection Eigen3::Eigen)
target_compile_features(tl_solid_model PUBLIC cxx_std_17)
target_compile_options(tl_solid_model PRIVATE -fno-fast-math -ffp-contract=off)
