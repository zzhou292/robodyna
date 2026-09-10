# Reuse the existing native recurrence core; no solver owner or CUDA dependency.
add_library(qeph_wall_recurrence_support STATIC MovingNativeProbe.cpp)
target_link_libraries(qeph_wall_recurrence_support PUBLIC qeph_recurrence_audit_core)
target_compile_features(qeph_wall_recurrence_support PUBLIC cxx_std_17)
target_compile_options(qeph_wall_recurrence_support PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(qeph_wall_recurrence_support_check
  MovingNativeProbeTest.cpp PowerGramCompositionTest.cpp)
target_link_libraries(qeph_wall_recurrence_support_check PRIVATE qeph_wall_recurrence_support GTest::gtest_main)
target_compile_features(qeph_wall_recurrence_support_check PRIVATE cxx_std_17)
target_compile_options(qeph_wall_recurrence_support_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME qeph_wall_recurrence_support_check COMMAND qeph_wall_recurrence_support_check)
set_tests_properties(qeph_wall_recurrence_support_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)

# Full native dictionary plus the existing host point law. The model/probe
# produces disposable qualification data, never production state or a clock.
include("${TL_ROOT}/lib_src/collision/NodalWallContact.cmake")
add_library(qeph_wall_recurrence_model STATIC
  WallRecurrenceModel.cpp ContactBranchOperator.cpp
  ContactNativeMap.cpp ContactBranchProbe.cpp)
target_link_libraries(qeph_wall_recurrence_model PUBLIC
  qeph_wall_recurrence_support tl_nodal_wall_contact)
target_compile_features(qeph_wall_recurrence_model PUBLIC cxx_std_17)
target_compile_options(qeph_wall_recurrence_model PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(qeph_wall_recurrence_model_check
  WallRecurrenceModelTest.cpp ContactBranchProbeTest.cpp)
target_link_libraries(qeph_wall_recurrence_model_check PRIVATE
  qeph_wall_recurrence_model GTest::gtest_main)
target_compile_features(qeph_wall_recurrence_model_check PRIVATE cxx_std_17)
target_compile_options(qeph_wall_recurrence_model_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME qeph_wall_recurrence_model_check COMMAND qeph_wall_recurrence_model_check)
set_tests_properties(qeph_wall_recurrence_model_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
