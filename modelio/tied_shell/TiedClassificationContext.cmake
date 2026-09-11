include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TiedAuxiliaryConstraints.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../rigid_part/RigidPartSource.cmake")
add_library(robo_dyna_tied_classification_context STATIC
  "${CMAKE_CURRENT_LIST_DIR}/TiedClassificationContext.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/classification/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/classification/ReadSet.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/classification/SourceRoles.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/classification/Wall.cpp")
target_link_libraries(robo_dyna_tied_classification_context PUBLIC
  robo_dyna_tied_auxiliary_constraints robo_dyna_rigid_part_source)
target_compile_features(robo_dyna_tied_classification_context PUBLIC cxx_std_17)
target_compile_options(robo_dyna_tied_classification_context PRIVATE -fno-fast-math -ffp-contract=off)
