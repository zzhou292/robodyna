// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_utest/qualification/tied_cin_runtime/OwnerFixture.h"
#include "Packet.h"

namespace tl::fea::cin_parallel_test {
namespace old = cin_runtime_test;
namespace {
void Capture(FENodalState& owner, const NodalTrialToken& token, std::vector<double>& values) {
  const auto n = values.size()/6;
  NodalPreparedView receipt;
  ASSERT_EQ(owner.CopyPreparedForceStage(token,
    {values.data(), values.data()+3*n, n, nullptr, 0}, &receipt).status, NodalStatus::Ok);
}
}
TEST(CinParallelOwner, OrdinaryFailureDiscardsEveryAcceptedFieldAndRetryKeepsCaptureAndSoleSelector) {
  old::Fixture f;
  auto config = f.Config();
  config.capture_force_stage_accelerations = true;
  FENodalState owner;
  ASSERT_EQ(owner.Initialize(config, f.Kinematics(), f.inverse.data(), f.Dofs(), f.Startup()).status,
    NodalStatus::Ok);
  const auto allocations = owner.allocations();
  const auto n = f.mass.size();
  old::Snapshot before(n, f.rows.size()), clean(n, f.rows.size()), retry(n, f.rows.size());
  std::vector<double> clean_capture(6*n), retry_capture(6*n);
  for (unsigned epoch=0; epoch<2; ++epoch) {
    SCOPED_TRACE(epoch);
    NodalStamp before_stamp, after_stamp;
    old::Accepted(owner, before, before_stamp);
    NodalTrialToken token;
    NodalAssemblyView assembly;
    NodalCinAssemblyView cin;
    old::Fill(owner, f, token, assembly, cin);
    ASSERT_EQ(owner.SealAssembly(token).status, NodalStatus::Ok);
    ASSERT_EQ(AdvanceStaggeredCin(owner, token, old::Admission(assembly)).status, NodalStatus::Ok);
    NodalPreparedView receipt;
    ASSERT_EQ(owner.CopyPrepared(token, clean.Nodes(), &receipt).status, NodalStatus::Ok);
    ASSERT_EQ(owner.CopyPreparedCin(token, clean.Cin(), &receipt).status, NodalStatus::Ok);
    Capture(owner, token, clean_capture);
    EXPECT_EQ(receipt.kick_dt, epoch==0 ? H/2 : H);
    owner.Discard();
    std::size_t failing_node = n;
    for (std::size_t i=0; i<n; ++i) if (!f.dependent[i]) { failing_node=i; break; }
    ASSERT_LT(failing_node, n);
    const auto original_couple = f.load[3*n+failing_node];
    f.load[3*n+failing_node] = 1e12;
    old::Fill(owner, f, token, assembly, cin);
    ASSERT_EQ(owner.SealAssembly(token).status, NodalStatus::Ok);
    const auto failure = AdvanceStaggeredCin(owner, token, old::Admission(assembly));
    EXPECT_EQ(failure.status, NodalStatus::StepTooLarge);
    EXPECT_EQ(failure.node, failing_node);
    const auto previous_nodes = retry.nodes;
    receipt.owner_id = 993;
    EXPECT_NE(owner.CopyPrepared(token, retry.Nodes(), &receipt).status, NodalStatus::Ok);
    EXPECT_EQ(retry.nodes, previous_nodes);
    EXPECT_EQ(receipt.owner_id, 993u);
    old::Accepted(owner, retry, after_stamp);
    old::Same(before, retry);
    EXPECT_TRUE(trial_identity::SameStamp(before_stamp, after_stamp));
    f.load[3*n+failing_node] = original_couple;
    old::Fill(owner, f, token, assembly, cin);
    ASSERT_EQ(owner.SealAssembly(token).status, NodalStatus::Ok);
    ASSERT_EQ(AdvanceStaggeredCin(owner, token, old::Admission(assembly)).status, NodalStatus::Ok);
    ASSERT_EQ(owner.CopyPrepared(token, retry.Nodes(), &receipt).status, NodalStatus::Ok);
    ASSERT_EQ(owner.CopyPreparedCin(token, retry.Cin(), &receipt).status, NodalStatus::Ok);
    Capture(owner, token, retry_capture);
    old::Same(clean, retry);
    SameDoubles(clean_capture, retry_capture);
    old::Complete(owner, token, assembly);
    ASSERT_EQ(owner.Commit(token).status, NodalStatus::Ok);
    old::Accepted(owner, retry, after_stamp);
    old::Same(clean, retry);
    EXPECT_EQ(after_stamp.epoch, epoch+1);
    EXPECT_EQ(owner.allocations().device_bytes, allocations.device_bytes);
    EXPECT_EQ(owner.allocations().device_allocations, allocations.device_allocations);
  }
}
} // namespace tl::fea::cin_parallel_test
