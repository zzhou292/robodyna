// SPDX-License-Identifier: AGPL-3.0-or-later
// Reuse the actual owner fixture and complete snapshot/capture helpers; this
// translation unit also keeps its established ordinary-failure regression.
#include "../cin_parallel_ordinary/OwnerTest.cu"

namespace tl::fea::cin_parallel_test {
TEST(CinParallelGroupOwner, LaterRigidFailurePreservesAllAcceptedSlabsAndAllocationThenRetry) {
  old::Fixture f;
  FENodalState owner;
  const auto initialized = old::Initialize(owner, f, true, true);
  ASSERT_EQ(initialized.status, NodalStatus::Ok) << initialized.message;
  const auto allocation = owner.allocations();
  const auto n = f.m.size();
  const auto r = f.cin_model.rows().count;
  const auto failing_node = f.binding.members()[f.binding.members().size()-1].domain_node;
  auto loads = old::Loads(f);
  fields::Snapshot before(n, r), clean(n, r), retry(n, r);
  old::Snapshot before_groups(n), after_groups(n);
  std::vector<double> clean_capture(6*n+12), retry_capture(6*n+12);
  std::vector<double> clean_rigid(36), retry_rigid(36);
  for (unsigned epoch = 0; epoch < 2; ++epoch) {
    NodalStamp before_stamp, after_stamp;
    fields::Accepted(owner, before, before_stamp);
    before_groups.Read(owner);
    NodalTrialToken token;
    NodalAssemblyView assembly;
    old::Begin(owner, f, loads, true, token, assembly);
    ASSERT_EQ(old::Advance(owner, token, assembly, true).status, NodalStatus::Ok);
    NodalPreparedView receipt;
    Prepared(owner, token, clean, clean_rigid, receipt);
    Capture(owner, token, clean_capture);
    owner.Discard();
    const auto saved = loads[3*n+failing_node];
    loads[3*n+failing_node] = 1e15;
    old::Begin(owner, f, loads, true, token, assembly);
    EXPECT_EQ(old::Advance(owner, token, assembly, true).status, NodalStatus::StepTooLarge);
    fields::Accepted(owner, retry, after_stamp);
    after_groups.Read(owner);
    fields::Same(before, retry);
    old::Same(before_groups, after_groups);
    EXPECT_TRUE(trial_identity::SameStamp(before_stamp, after_stamp));
    const auto prior_nodes = retry.nodes;
    receipt.owner_id = 991;
    EXPECT_NE(owner.CopyPrepared(token, retry.Nodes(), &receipt).status, NodalStatus::Ok);
    EXPECT_EQ(retry.nodes, prior_nodes);
    EXPECT_EQ(receipt.owner_id, 991u);
    loads[3*n+failing_node] = saved;
    old::Begin(owner, f, loads, true, token, assembly);
    ASSERT_EQ(old::Advance(owner, token, assembly, true).status, NodalStatus::Ok);
    Prepared(owner, token, retry, retry_rigid, receipt);
    Capture(owner, token, retry_capture);
    fields::Same(clean, retry);
    SameDoubles(clean_rigid, retry_rigid);
    SameDoubles(clean_capture, retry_capture);
    old::Commit(owner, token, assembly);
    EXPECT_EQ(owner.allocations().device_bytes, allocation.device_bytes);
    EXPECT_EQ(owner.allocations().device_allocations, allocation.device_allocations);
  }
}
} // namespace tl::fea::cin_parallel_test
