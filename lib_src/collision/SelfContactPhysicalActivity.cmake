include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SelfContactActiveUseBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../elements/ShellBatchPublication.cmake")

add_library(tl_self_contact_physical_activity_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_physical_activity/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_physical_activity/Selection.h"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_physical_activity/Values.cpp")
target_link_libraries(tl_self_contact_physical_activity_values PUBLIC
  tl_self_contact_active_uses
  tl_shell_batch_publication)
target_compile_features(tl_self_contact_physical_activity_values PUBLIC
  cxx_std_17)
target_compile_options(tl_self_contact_physical_activity_values PRIVATE
  -fno-fast-math -ffp-contract=off)

add_library(tl_self_contact_physical_activity STATIC
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_physical_activity/Activity.cpp")
target_link_libraries(tl_self_contact_physical_activity PUBLIC
  tl_self_contact_physical_activity_values
  tl_shell_batch_publication)
target_compile_features(tl_self_contact_physical_activity PUBLIC cxx_std_17)
target_compile_options(tl_self_contact_physical_activity PRIVATE
  -fno-fast-math -ffp-contract=off)
