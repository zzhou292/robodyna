// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../cin_parallel_ordinary/OwnerTest.cu"
#include "lib_src/constraints/tied_shell/TiedPatchForce.h"
#include <cfloat>

namespace tl::fea::cin_parallel_test {
TEST(CinPreparedForceRowsOwner, LateTransferFailureKeepsAcceptedStateCaptureAllocationAndRetry) {
  old::Fixture fixture;
  FENodalState owner;
  const auto initialized = old::Initialize(owner, fixture, true, true);
  ASSERT_EQ(initialized.status, NodalStatus::Ok) << initialized.message;
  const auto allocations = owner.allocations();
  const auto nodes = fixture.m.size();
  const auto rows = fixture.cin_model.rows();
  ASSERT_GT(rows.count, 1u);
  const auto& row = rows.data[rows.count-1];
  const auto secondary = row.secondary_domain_node;
  auto loads = old::Loads(fixture);
  fields::Snapshot before(nodes, rows.count), clean(nodes, rows.count), retry(nodes, rows.count);
  old::Snapshot before_groups(nodes), after_groups(nodes);
  std::vector<double> clean_capture(6*nodes+12), retry_capture(6*nodes+12);
  std::vector<double> clean_rigid(36), retry_rigid(36);
  for (unsigned epoch = 0; epoch < 2; ++epoch) {
    NodalStamp before_stamp, after_stamp;
    fields::Accepted(owner, before, before_stamp);
    before_groups.Read(owner);
    NodalTrialToken token;
    NodalAssemblyView assembly;
    old::Begin(owner, fixture, loads, true, token, assembly);
    ASSERT_EQ(old::Advance(owner, token, assembly, true).status, NodalStatus::Ok);
    NodalPreparedView receipt;
    Prepared(owner, token, clean, clean_rigid, receipt);
    Capture(owner, token, clean_capture);
    owner.Discard();

    // Select a finite load which the actual current patch proves overflows in
    // transfer, after input admission and every earlier row's application.
    using namespace constraints::tied_shell;
    PatchInput geometry;
    geometry.secondary_position = cin::detail::ReadXyz(before.nodes.data(), secondary);
    for (unsigned slot = 0; slot < 4; ++slot)
      geometry.master_position[slot] = cin::detail::ReadXyz(before.nodes.data(), row.master_domain_nodes[slot]);
    Patch patch;
    ASSERT_EQ(PreparePatch(geometry, patch), Status::Success);
    unsigned selected = 6;
    for (unsigned axis = 0; axis < 6 && selected == 6; ++axis) {
      double values[6]{};
      values[axis] = DBL_MAX;
      MasterLoads transferred;
      if (TransferLoad(patch, {{values[0],values[1],values[2]},
          {values[3],values[4],values[5]}}, transferred) == Status::NonfiniteResult) selected = axis;
    }
    ASSERT_LT(selected, 6u);
    const auto saved = loads[selected*nodes+secondary];
    loads[selected*nodes+secondary] = DBL_MAX;
    old::Begin(owner, fixture, loads, true, token, assembly);
    const auto failure = old::Advance(owner, token, assembly, true);
    EXPECT_EQ(failure.status, NodalStatus::InvalidOutput);
    EXPECT_EQ(failure.node, secondary);
    fields::Accepted(owner, retry, after_stamp);
    after_groups.Read(owner);
    fields::Same(before, retry);
    old::Same(before_groups, after_groups);
    EXPECT_TRUE(trial_identity::SameStamp(before_stamp, after_stamp));
    const auto prior_nodes = retry.nodes;
    receipt.owner_id = 993;
    EXPECT_NE(owner.CopyPrepared(token, retry.Nodes(), &receipt).status, NodalStatus::Ok);
    EXPECT_EQ(retry.nodes, prior_nodes);
    EXPECT_EQ(receipt.owner_id, 993u);
    std::array<NodalRigidGroupAccelerationSnapshot, 2> rejected_groups;
    const auto prior_capture = retry_capture;
    EXPECT_NE(owner.CopyPreparedForceStage(token,
        {retry_capture.data(), retry_capture.data()+3*nodes, nodes,
         rejected_groups.data(), rejected_groups.size()}, &receipt).status, NodalStatus::Ok);
    EXPECT_EQ(retry_capture, prior_capture);

    loads[selected*nodes+secondary] = saved;
    old::Begin(owner, fixture, loads, true, token, assembly);
    ASSERT_EQ(old::Advance(owner, token, assembly, true).status, NodalStatus::Ok);
    Prepared(owner, token, retry, retry_rigid, receipt);
    Capture(owner, token, retry_capture);
    fields::Same(clean, retry);
    SameDoubles(clean_rigid, retry_rigid);
    SameDoubles(clean_capture, retry_capture);
    old::Commit(owner, token, assembly);
    EXPECT_EQ(owner.allocations().device_bytes, allocations.device_bytes);
    EXPECT_EQ(owner.allocations().device_allocations, allocations.device_allocations);
  }
}
} // namespace tl::fea::cin_parallel_test
