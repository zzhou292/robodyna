include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../assembly/NodalNodeDomain.cmake")
add_library(tl_beam18_model STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Model.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/model/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/model/Parents.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/model/Identity.cpp")
target_link_libraries(tl_beam18_model PUBLIC tl_nodal_node_domain)
target_include_directories(tl_beam18_model PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../../..")
target_compile_features(tl_beam18_model PUBLIC cxx_std_17)
target_compile_options(tl_beam18_model PRIVATE -fno-fast-math -ffp-contract=off)
