include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SelfContactForceAssembly.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/SelfContactBroadphase.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/SelfContactFilterCertificates.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/SelfContactCurrentRegularity.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/SelfContactPhysicalActivity.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/RepresentedIntervalCrossing.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../elements/ShellBatchPublication.cmake")

add_library(tl_self_contact_transaction STATIC
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_transaction/Arena.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_transaction/Limits.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_transaction/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_transaction/Values.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_transaction/Streaming.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_transaction/TaskMask.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_transaction/RigidSweep.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_transaction/Source.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_transaction/Transaction.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_transaction/Candidate.cpp")
target_link_libraries(tl_self_contact_transaction PUBLIC
  tl_self_contact_force_assembly
  tl_self_contact_broadphase
  tl_self_contact_filter_certificates
  tl_self_contact_current_regularity
  tl_self_contact_physical_activity
  tl_represented_interval_crossing
  tl_shell_batch_publication)
target_compile_features(tl_self_contact_transaction PUBLIC cxx_std_17)
target_compile_options(tl_self_contact_transaction PRIVATE
  -fno-fast-math -ffp-contract=off)
