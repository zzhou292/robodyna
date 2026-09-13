include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../modelio/self_contact/OriginalSelection.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/SelfContactActiveUseBinding.cmake")
add_library(robo_dyna_vehicle_self_contact_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SelectedSelfContactSource.cpp")
target_link_libraries(robo_dyna_vehicle_self_contact_values PUBLIC
  robo_dyna_original_self_contact_selection tl_self_contact_active_uses)
target_compile_features(robo_dyna_vehicle_self_contact_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_self_contact_values PRIVATE
  -fno-fast-math -ffp-contract=off)
