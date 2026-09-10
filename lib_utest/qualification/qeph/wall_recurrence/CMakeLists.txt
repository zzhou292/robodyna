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

# Full-state analysis consumes retained derivatives; it owns neither the native
# capture loop nor any production mechanics state.
add_library(qeph_wall_recurrence_analysis STATIC
  WallStateMetric.cpp WallSwitchingSchedule.cpp WallRecurrenceSpectrum.cpp
  WallSequenceAnalysis.cpp WallStateIdentities.cpp
  WallRecurrenceAnalysis.cpp WallAnalysisComparison.cpp)
target_link_libraries(qeph_wall_recurrence_analysis PUBLIC qeph_wall_recurrence_model)
target_compile_features(qeph_wall_recurrence_analysis PUBLIC cxx_std_17)
target_compile_options(qeph_wall_recurrence_analysis PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(qeph_wall_recurrence_analysis_check
  WallMetricScheduleTest.cpp WallSpectrumTest.cpp WallIdentityComparisonTest.cpp)
target_link_libraries(qeph_wall_recurrence_analysis_check PRIVATE
  qeph_wall_recurrence_analysis GTest::gtest_main)
target_compile_features(qeph_wall_recurrence_analysis_check PRIVATE cxx_std_17)
target_compile_options(qeph_wall_recurrence_analysis_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME qeph_wall_recurrence_analysis_check COMMAND qeph_wall_recurrence_analysis_check)
set_tests_properties(qeph_wall_recurrence_analysis_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)

# Capture stays numerical; the separate create-only writer reuses application
# artifact IO and the existing bounded recurrence JSON encoder.
add_library(qeph_wall_raw_capture STATIC WallRawCapture.cpp)
target_link_libraries(qeph_wall_raw_capture PUBLIC qeph_wall_recurrence_model)
target_compile_features(qeph_wall_raw_capture PUBLIC cxx_std_17)
target_compile_options(qeph_wall_raw_capture PRIVATE -fno-fast-math -ffp-contract=off)
add_library(qeph_wall_raw_report STATIC
  WallRawJson.cpp WallRawModel.cpp WallRawContact.cpp WallRawProbes.cpp
  WallRawReport.cpp WallRawWriter.cpp WallRawInventory.cpp)
target_link_libraries(qeph_wall_raw_report PUBLIC qeph_wall_raw_capture qeph_recurrence_report)
target_compile_features(qeph_wall_raw_report PUBLIC cxx_std_17)
target_compile_options(qeph_wall_raw_report PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(qeph_wall_raw_check
  WallRawCaptureTest.cpp WallRawReportTest.cpp WallRawContactReportTest.cpp)
target_link_libraries(qeph_wall_raw_check PRIVATE qeph_wall_raw_report GTest::gtest_main)
target_compile_features(qeph_wall_raw_check PRIVATE cxx_std_17)
target_compile_options(qeph_wall_raw_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME qeph_wall_raw_check COMMAND qeph_wall_raw_check)
set_tests_properties(qeph_wall_raw_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)

add_executable(qeph_wall_raw wall_raw_main.cpp)
target_link_libraries(qeph_wall_raw PRIVATE qeph_wall_raw_report)
target_compile_features(qeph_wall_raw PRIVATE cxx_std_17)
target_compile_options(qeph_wall_raw PRIVATE -fno-fast-math -ffp-contract=off)
