include_guard(GLOBAL)
# This concrete runtime requires the GPU maintenance target even when a host
# source module has already included its values-only build description.
set(TYPE25_SEARCH_CUDA ON)
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25Transaction.cmake")
add_library(robo_dyna_native_contact_contribution STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Contribution.cpp" "${CMAKE_CURRENT_LIST_DIR}/Error.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Group.cpp")
get_filename_component(_native_contribution_app "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
target_include_directories(robo_dyna_native_contact_contribution PUBLIC "${_native_contribution_app}")
target_link_libraries(robo_dyna_native_contact_contribution PUBLIC tl_radioss_type25_transaction)
target_compile_features(robo_dyna_native_contact_contribution PUBLIC cxx_std_17)
target_compile_options(robo_dyna_native_contact_contribution PRIVATE -fno-fast-math -ffp-contract=off)
