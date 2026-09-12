// SPDX-License-Identifier: MIT
#include "OwnerSupport.h"

namespace qeph_activity_test {
TEST(QephMappedActivityCuda,LastForceFailurePreservesAcceptedOutputAndCandidateRollbackRetry) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  fe::NodalTrialToken token;
  fe::NodalPreparedView view;
  fe::ShellPhysicalDiagnostics candidate;
  ASSERT_TRUE(rig.Prepare(token,view,candidate));
  ASSERT_TRUE(Commit(rig,token,view,candidate));
  for (bool prepared : {false,true}) {
    if (prepared) ASSERT_TRUE(rig.Prepare(token,view,candidate));
    Oracle oracle;
    ASSERT_NO_FATAL_FAILURE(ReadOracle(rig,oracle,prepared ? &candidate.qeph : nullptr));
    auto* device = static_cast<q::ForceTrial*>(const_cast<void*>(transfers.force_source));
    ASSERT_NE(device,nullptr);
    const auto stream = transfers.stream;
    const auto n = oracle.staging.size();
    const auto original = oracle.staging.back();
    oracle.staging.back().kinematics.projection_inverse[3] = std::numeric_limits<double>::quiet_NaN();
    const auto expected = serial::ValidateMappedSections(oracle,prepared ? 1 : 0);
    ASSERT_EQ(expected.status,q::BatchStatus::NonfiniteResult);
    ASSERT_EQ(cudaMemcpyAsync(device+n-1,&oracle.staging.back(),sizeof(q::ForceTrial),cudaMemcpyHostToDevice,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    std::vector<std::uint8_t> flags(n,19);
    q::BatchDiagnostics diagnostics;
    const auto untouched = qt_mapped_test::Bytes(diagnostics);
    const auto accepted = rig.owner.accepted();
    const auto report = prepared
        ? rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n)
        : rig.qeph.CopyAcceptedParentActivity(accepted,flags.data(),n,&diagnostics);
    SameReport(report,expected);
    EXPECT_EQ(flags,std::vector<std::uint8_t>(n,19));
    EXPECT_EQ(qt_mapped_test::Bytes(diagnostics),untouched);
    EXPECT_TRUE(fe::trial_identity::SameStamp(accepted,rig.owner.accepted()));
    ASSERT_EQ(cudaMemcpyAsync(device+n-1,&original,sizeof(original),cudaMemcpyHostToDevice,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    if (prepared) {
      EXPECT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n).status,
          q::BatchStatus::StaleTrial);
      Discard(rig);
      ASSERT_TRUE(rig.Prepare(token,view,candidate));
      ASSERT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n).status,
          q::BatchStatus::Success);
      ASSERT_TRUE(Commit(rig,token,view,candidate));
    } else {
      ASSERT_EQ(rig.qeph.CopyAcceptedParentActivity(accepted,flags.data(),n,&diagnostics).status,q::BatchStatus::Success);
    }
  }
}
} // namespace qeph_activity_test
