include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/CorrectedNodalSource.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25GapSource.cmake")
if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_src/elements/beam18/PropertyArea.h")
  message(FATAL_ERROR "Gap operands require the separate native beam property-area capability; owning qualification is a separate gate")
endif()
add_library(robo_dyna_contact_gap_operands STATIC
  "${CMAKE_CURRENT_LIST_DIR}/ContactGapOperands.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/gap_operands/Binding.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/gap_operands/Shells.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/gap_operands/Lines.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/gap_operands/Springs.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/gap_operands/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/gap_operands/Digest.cpp")
target_link_libraries(robo_dyna_contact_gap_operands PUBLIC robo_dyna_vehicle_corrected_nodal_source tl_radioss_type25_gap_source)
target_compile_features(robo_dyna_contact_gap_operands PUBLIC cxx_std_17)
target_compile_options(robo_dyna_contact_gap_operands PRIVATE -fno-fast-math -ffp-contract=off)
