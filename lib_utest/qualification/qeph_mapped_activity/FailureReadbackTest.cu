// SPDX-License-Identifier: MIT
#include "OwnerSupport.h"

namespace qeph_activity_test {
namespace {
template<class T> bool Upload(T* device, const T& value, cudaStream_t stream) {
  return cudaMemcpyAsync(device,&value,sizeof(T),cudaMemcpyHostToDevice,stream) == cudaSuccess &&
      cudaStreamSynchronize(stream) == cudaSuccess;
}
}
TEST(QephFailureActivityCuda,FreshSectionFailureAndForcePhasesMatchFullReadAndRetry) {
  std::array<q::BatchReport,2> frozen_reports;
  for (bool prepared : {false,true}) {
    for (unsigned section_fault = 0; section_fault < 2; ++section_fault) {
      SCOPED_TRACE(prepared);
      SCOPED_TRACE(section_fault);
      Rig rig(true);
      ASSERT_TRUE(rig.Initialize());
      fe::NodalTrialToken token;
      fe::NodalPreparedView view;
      fe::ShellPhysicalDiagnostics candidate;
      ASSERT_TRUE(rig.Prepare(token,view,candidate));
      ASSERT_TRUE(Commit(rig,token,view,candidate));
      if (prepared) ASSERT_TRUE(rig.Prepare(token,view,candidate));
      Oracle oracle;
      ASSERT_NO_FATAL_FAILURE(ReadOracle(rig,oracle,prepared ? &candidate.qeph : nullptr));
      const auto n = oracle.staging.size();
      ASSERT_GT(n,1u);
      auto* failure = static_cast<fe::ShellBatchFailureState*>(const_cast<void*>(transfers.failure_source));
      auto* force = static_cast<q::ForceTrial*>(const_cast<void*>(transfers.force_source));
      auto* plastic = static_cast<fe::ShellBatchSectionState*>(const_cast<void*>(transfers.plastic_source));
      ASSERT_NE(failure,nullptr);
      ASSERT_NE(force,nullptr);
      ASSERT_NE(plastic,nullptr);
      const auto stream = transfers.stream;
      fe::ShellBatchSectionState saved_section;
      ASSERT_EQ(cudaMemcpyAsync(&saved_section,plastic+n-1,sizeof(saved_section),cudaMemcpyDeviceToHost,stream),cudaSuccess);
      ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
      ASSERT_NE(oracle.histories.sections.back().plastic(),nullptr);
      auto bad_section = saved_section;
      bad_section.cumulative_plastic_work_J = std::numeric_limits<double>::quiet_NaN();
      auto bad_force = oracle.staging.front();
      bad_force.internal_force[0].x = std::numeric_limits<double>::quiet_NaN();
      auto bad_failure = oracle.histories.failures.back();
      *reinterpret_cast<unsigned char*>(&bad_failure.active) = 2;
      ASSERT_TRUE(Upload(force,bad_force,stream));
      ASSERT_TRUE(Upload(failure+n-1,bad_failure,stream));
      if (section_fault) ASSERT_TRUE(Upload(plastic+n-1,bad_section,stream));
      const auto stamp = rig.owner.accepted();
      std::vector<std::uint8_t> flags(n,19);
      q::BatchDiagnostics diagnostics;
      const auto untouched = qt_mapped_test::Bytes(diagnostics);
      if (!prepared) {
        std::vector<fe::ShellBatchFailureState> complete(n);
        frozen_reports[section_fault] = rig.qeph.CopyAcceptedFailureHistory(stamp,complete.data(),n,&diagnostics);
        ASSERT_EQ(frozen_reports[section_fault].status,q::BatchStatus::NonfiniteResult);
      }
      const auto report = prepared
          ? rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n)
          : rig.qeph.CopyAcceptedParentActivity(stamp,flags.data(),n,&diagnostics);
      SameReport(report,frozen_reports[section_fault]);
      EXPECT_EQ(flags,std::vector<std::uint8_t>(n,19));
      EXPECT_EQ(qt_mapped_test::Bytes(diagnostics),untouched);
      EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,rig.owner.accepted()));
      ASSERT_TRUE(Upload(force,oracle.staging.front(),stream));
      ASSERT_TRUE(Upload(failure+n-1,oracle.histories.failures.back(),stream));
      ASSERT_TRUE(Upload(plastic+n-1,saved_section,stream));
      if (prepared) {
        EXPECT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n).status,
            q::BatchStatus::StaleTrial);
        Discard(rig);
        ASSERT_TRUE(rig.Prepare(token,view,candidate));
        ASSERT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n).status,
            q::BatchStatus::Success);
        ASSERT_TRUE(Commit(rig,token,view,candidate));
      } else {
        ASSERT_EQ(rig.qeph.CopyAcceptedParentActivity(stamp,flags.data(),n,&diagnostics).status,q::BatchStatus::Success);
      }
      Oracle repeated;
      ASSERT_NO_FATAL_FAILURE(ReadOracle(rig,repeated));
    }
  }
}

TEST(QephFailureActivityCuda,CorruptedCompactFailureFlagsRejectBeforeForceAndKeepOutputs) {
  for (bool prepared : {false,true}) {
    for (std::uint8_t raw : {std::uint8_t(2),std::uint8_t(255)}) {
      Rig rig;
      ASSERT_TRUE(rig.Initialize());
      fe::NodalTrialToken token;
      fe::NodalPreparedView view;
      fe::ShellPhysicalDiagnostics candidate;
      if (prepared) ASSERT_TRUE(rig.Prepare(token,view,candidate));
      const auto n = rig.fixture.physical.shells()->qeph_count();
      std::vector<std::uint8_t> flags(n,19);
      q::BatchDiagnostics diagnostics;
      const auto untouched = qt_mapped_test::Bytes(diagnostics);
      Watch(n);
      transfers.corrupt_compact_call = 1;
      transfers.corrupt_flag = raw;
      const auto report = prepared
          ? rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n)
          : rig.qeph.CopyAcceptedParentActivity(rig.owner.accepted(),flags.data(),n,&diagnostics);
      EXPECT_EQ(report.status,q::BatchStatus::NonfiniteResult);
      EXPECT_STREQ(report.message,"Failure sidecar state disagrees with its declared policy/saved section");
      EXPECT_EQ(transfers.compact_calls,1u);
      EXPECT_EQ(transfers.calls,3u);
      transfers.enabled = false;
      EXPECT_EQ(flags,std::vector<std::uint8_t>(n,19));
      EXPECT_EQ(qt_mapped_test::Bytes(diagnostics),untouched);
      if (prepared) {
        EXPECT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n).status,
            q::BatchStatus::StaleTrial);
        Discard(rig);
        ASSERT_TRUE(rig.Prepare(token,view,candidate));
        ASSERT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n).status,
            q::BatchStatus::Success);
      } else {
        ASSERT_EQ(rig.qeph.CopyAcceptedParentActivity(rig.owner.accepted(),flags.data(),n,&diagnostics).status,
            q::BatchStatus::Success);
      }
      Discard(rig);
    }
  }
}

TEST(QephFailureActivityCuda,FailurePacketCopyErrorPoisonsWithoutPublishing) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  fe::NodalTrialToken token;
  fe::NodalPreparedView view;
  fe::ShellPhysicalDiagnostics candidate;
  ASSERT_TRUE(rig.Prepare(token,view,candidate));
  const auto n = rig.fixture.physical.shells()->qeph_count();
  const auto stamp = rig.owner.accepted();
  std::vector<std::uint8_t> flags(n,19);
  Watch(n);
  transfers.fail_copy = 3;
  EXPECT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n).status,
      q::BatchStatus::DeviceFailure);
  EXPECT_EQ(transfers.calls,3u);
  transfers.enabled = false;
  EXPECT_EQ(flags,std::vector<std::uint8_t>(n,19));
  EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,rig.owner.accepted()));
  q::BatchDiagnostics diagnostics;
  EXPECT_EQ(rig.qeph.CopyAcceptedParentActivity(stamp,flags.data(),n,&diagnostics).status,q::BatchStatus::DeviceFailure);
  Discard(rig);
}
} // namespace qeph_activity_test
