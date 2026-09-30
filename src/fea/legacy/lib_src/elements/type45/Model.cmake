include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../constraints/NodalRigidAssemblyBinding.cmake")
add_library(tl_type45_model STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Model.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/model/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/model/Mapping.cpp")
target_link_libraries(tl_type45_model PUBLIC tl_nodal_rigid_assembly_binding)
target_compile_features(tl_type45_model PUBLIC cxx_std_17)
target_compile_options(tl_type45_model PRIVATE -fno-fast-math -ffp-contract=off)
