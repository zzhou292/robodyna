include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../modelio/tied_shell/TiedShellSearchGeometry.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/constraints/tied_shell/search/TiedSearchDriverValues.cmake")
add_library(robo_dyna_tied_assessment_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/tied_search/CanonicalPayload.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tied_search/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tied_search/Inputs.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tied_search/Associations.cpp")
target_link_libraries(robo_dyna_tied_assessment_values PUBLIC robo_dyna_tied_search_geometry tl_tied_search_driver_values)
target_compile_features(robo_dyna_tied_assessment_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_tied_assessment_values PRIVATE -fno-fast-math -ffp-contract=off)
