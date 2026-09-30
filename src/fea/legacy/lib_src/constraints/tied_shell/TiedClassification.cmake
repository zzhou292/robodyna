if(NOT TARGET tl_tied_shell_classification)
  add_library(tl_tied_shell_classification STATIC
    "${CMAKE_CURRENT_LIST_DIR}/TiedClassification.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/TiedClassificationBudget.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/TiedClassificationChecks.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/TiedClassificationKinematics.cpp")
  get_filename_component(tl_classification_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
  target_include_directories(tl_tied_shell_classification PUBLIC "${tl_classification_root}")
  target_compile_features(tl_tied_shell_classification PUBLIC cxx_std_17)
endif()
