// SPDX-License-Identifier: MIT
#include "OwnerSupport.h"

namespace qeph_activity_test {
TEST(QephMixedActivityCuda,ValidButWrongRoleRemainsAfterCompleteForceValidation) {
  for (bool prepared : {false,true}) {
    for (bool bad_force : {false,true}) {
      Rig rig(true);
      ASSERT_TRUE(rig.Initialize());
      fe::NodalTrialToken token;
      fe::NodalPreparedView view;
      fe::ShellPhysicalDiagnostics candidate;
      if (prepared) ASSERT_TRUE(rig.Prepare(token,view,candidate));
      Oracle oracle;
      ASSERT_NO_FATAL_FAILURE(ReadOracle(rig,oracle,prepared ? &candidate.qeph : nullptr));
      ASSERT_NE(oracle.histories.sections.back().plastic(),nullptr);
      const auto n = oracle.staging.size();
      auto* force = static_cast<q::ForceTrial*>(const_cast<void*>(transfers.force_source));
      ASSERT_NE(force,nullptr);
      const auto stream = transfers.stream;
      auto altered = oracle.staging.front();
      if (bad_force) {
        altered.internal_force[0].x = std::numeric_limits<double>::quiet_NaN();
        ASSERT_EQ(cudaMemcpyAsync(force,&altered,sizeof(altered),cudaMemcpyHostToDevice,stream),cudaSuccess);
        ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
      }
      std::vector<std::uint8_t> flags(n,19);
      q::BatchDiagnostics diagnostics;
      Watch(n);
      transfers.corrupt_compact_call = 1;
      transfers.corrupt_flag = static_cast<std::uint8_t>(fe::ShellSectionLaw::RigidSkin);
      const auto report = prepared
          ? rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n)
          : rig.qeph.CopyAcceptedParentActivity(rig.owner.accepted(),flags.data(),n,&diagnostics);
      EXPECT_EQ(report.status,q::BatchStatus::NonfiniteResult);
      EXPECT_STREQ(report.message,bad_force ? "Mapped Qeph force cache differs from its source/endpoint role"
          : "Mapped Qeph typed section role differs");
      EXPECT_EQ(report.element,bad_force ? 0u : n-1);
      CheckTransfer();
      transfers.enabled = false;
      EXPECT_EQ(flags,std::vector<std::uint8_t>(n,19));
      ASSERT_EQ(cudaMemcpyAsync(force,&oracle.staging.front(),sizeof(altered),cudaMemcpyHostToDevice,stream),cudaSuccess);
      ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
      if (prepared) {
        Discard(rig);
        ASSERT_TRUE(rig.Prepare(token,view,candidate));
      }
      ASSERT_EQ((prepared
          ? rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n)
          : rig.qeph.CopyAcceptedParentActivity(rig.owner.accepted(),flags.data(),n,&diagnostics)).status,
          q::BatchStatus::Success);
      Discard(rig);
    }
  }
}

TEST(QephMixedActivityCuda,InvalidCompactRolesStopBeforeFailureAndRetryFreshly) {
  for (bool prepared : {false,true}) {
    for (std::uint8_t raw : {std::uint8_t(0),std::uint8_t(3),std::uint8_t(255)}) {
      Rig rig;
      ASSERT_TRUE(rig.Initialize());
      fe::NodalTrialToken token;
      fe::NodalPreparedView view;
      fe::ShellPhysicalDiagnostics candidate;
      if (prepared) ASSERT_TRUE(rig.Prepare(token,view,candidate));
      const auto n = rig.fixture.physical.shells()->qeph_count();
      const auto stamp = rig.owner.accepted();
      std::vector<std::uint8_t> flags(n,19);
      q::BatchDiagnostics diagnostics;
      const auto untouched = qt_mapped_test::Bytes(diagnostics);
      Watch(n);
      transfers.corrupt_compact_call = 1;
      transfers.corrupt_flag = raw;
      const auto report = prepared
          ? rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n)
          : rig.qeph.CopyAcceptedParentActivity(stamp,flags.data(),n,&diagnostics);
      EXPECT_EQ(report.status,q::BatchStatus::InvalidInput);
      EXPECT_STREQ(report.message,"Unsupported section readback law");
      EXPECT_EQ(transfers.calls,1u);
      EXPECT_EQ(transfers.compact_calls,1u);
      transfers.enabled = false;
      EXPECT_EQ(flags,std::vector<std::uint8_t>(n,19));
      EXPECT_EQ(qt_mapped_test::Bytes(diagnostics),untouched);
      EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,rig.owner.accepted()));
      if (prepared) {
        EXPECT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n).status,
            q::BatchStatus::StaleTrial);
        Discard(rig);
        ASSERT_TRUE(rig.Prepare(token,view,candidate));
      }
      Watch(n);
      ASSERT_EQ((prepared
          ? rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n)
          : rig.qeph.CopyAcceptedParentActivity(stamp,flags.data(),n,&diagnostics)).status,q::BatchStatus::Success);
      CheckTransfer();
      transfers.enabled = false;
      Discard(rig);
    }
  }
}

TEST(QephMixedActivityCuda,MixedPacketAliasAndCopyFailurePreservePublicOutputs) {
  for (bool fault : {false,true}) {
    Rig rig;
    ASSERT_TRUE(rig.Initialize());
    const auto n = rig.fixture.physical.shells()->qeph_count();
    std::vector<std::uint8_t> flags(n,19);
    q::BatchDiagnostics diagnostics;
    Watch(n);
    ASSERT_EQ(rig.qeph.CopyAcceptedParentActivity(rig.owner.accepted(),flags.data(),n,&diagnostics).status,
        q::BatchStatus::Success);
    auto* roles = static_cast<std::uint8_t*>(transfers.compact_destinations[0]);
    transfers.enabled = false;
    ASSERT_NE(roles,nullptr);
    EXPECT_EQ(rig.qeph.CopyAcceptedParentActivity(rig.owner.accepted(),roles,n,&diagnostics).status,
        q::BatchStatus::InvalidInput);
    fe::NodalTrialToken token;
    fe::NodalPreparedView view;
    fe::ShellPhysicalDiagnostics candidate;
    ASSERT_TRUE(rig.Prepare(token,view,candidate));
    EXPECT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,roles,n).status,
        q::BatchStatus::InvalidInput);
    flags.assign(n,19);
    Watch(n);
    if (fault) transfers.fail_copy = 1;
    const auto report = rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n);
    EXPECT_EQ(report.status,fault ? q::BatchStatus::DeviceFailure : q::BatchStatus::Success);
    transfers.enabled = false;
    if (fault) {
      EXPECT_EQ(flags,std::vector<std::uint8_t>(n,19));
      EXPECT_EQ(rig.qeph.CopyAcceptedParentActivity(rig.owner.accepted(),flags.data(),n,&diagnostics).status,
          q::BatchStatus::DeviceFailure);
    }
    Discard(rig);
  }
}
} // namespace qeph_activity_test
