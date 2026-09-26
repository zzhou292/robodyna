include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/ContactNodalSeed.cmake")
if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_src/collision/radioss_type25/source_nodal/OrderCertificate.cpp")
  message(FATAL_ERROR "Corrected source requires the independently qualified factor certificate capability")
endif()
add_library(robo_dyna_vehicle_corrected_nodal_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/CorrectedNodalSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_correction/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_correction/Context.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_correction/Sharing.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_correction/Inputs.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_correction/Digest.cpp")
target_link_libraries(robo_dyna_vehicle_corrected_nodal_source PUBLIC robo_dyna_vehicle_contact_nodal_seed)
target_compile_features(robo_dyna_vehicle_corrected_nodal_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_corrected_nodal_source PRIVATE -fno-fast-math -ffp-contract=off)
