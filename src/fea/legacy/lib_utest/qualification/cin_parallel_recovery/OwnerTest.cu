// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_utest/qualification/rigid_assembly_owner/OwnerFixture.h"
#include "lib_utest/qualification/tied_cin_runtime/OwnerFixture.h"
#include "../cin_parallel_ordinary/Packet.h"

namespace tl::fea::cin_parallel_test {
namespace old = rigid_assembly_owner_test;
namespace fields = cin_runtime_test;
namespace {
void Capture(FENodalState& owner, const NodalTrialToken& token, std::vector<double>& values) {
  const auto n = (values.size()-12)/6;
  std::array<NodalRigidGroupAccelerationSnapshot, 2> groups;
  NodalPreparedView receipt;
  ASSERT_EQ(owner.CopyPreparedForceStage(token,
    {values.data(), values.data()+3*n, n, groups.data(), groups.size()}, &receipt).status, NodalStatus::Ok);
  for (unsigned g=0; g<groups.size(); ++g) {
    const auto& group = groups[g];
    const double state[]{group.acceleration.x, group.acceleration.y, group.acceleration.z,
      group.angular_acceleration.x, group.angular_acceleration.y, group.angular_acceleration.z};
    std::copy(std::begin(state), std::end(state), values.begin()+6*n+6*g);
  }
}
void Prepared(FENodalState& owner, const NodalTrialToken& token, fields::Snapshot& out,
    std::vector<double>& rigid_values, NodalPreparedView& receipt) {
  ASSERT_EQ(owner.CopyPrepared(token, out.Nodes(), &receipt).status, NodalStatus::Ok);
  ASSERT_EQ(owner.CopyPreparedCin(token, out.Cin(), &receipt).status, NodalStatus::Ok);
  std::array<NodalRigidGroupSnapshot, 2> groups;
  ASSERT_EQ(owner.CopyPreparedRigidGroups(token, {groups.data(), groups.size()}, &receipt).status,
    NodalStatus::Ok);
  for (unsigned g=0; g<groups.size(); ++g) rigid::WriteGroupState(rigid_values.data()+18*g, groups[g].state);
}
}
TEST(CinParallelOwner, RecoveredSecondaryFailureDiscardsEveryAcceptedFieldAndRetryKeepsCaptureAndSoleSelector) {
  // Capture is admitted only by actual rigid-group/binding startup. Reuse the
  // existing physical PART/plain+CIN fixture instead of weakening that profile.
  old::Fixture f;
  FENodalState owner;
  const auto initialized = old::Initialize(owner, f, true, true);
  ASSERT_EQ(initialized.status, NodalStatus::Ok) << initialized.message;
  ASSERT_EQ(f.binding.groups().size(), 2u);
  const auto allocations = owner.allocations();
  const auto n = f.m.size();
  const auto r = f.cin_model.rows().count;
  fields::Snapshot before(n, r), clean(n, r), retry(n, r);
  old::Snapshot before_groups(n), after_groups(n);
  std::vector<double> clean_capture(6*n+12), retry_capture(6*n+12);
  std::vector<double> clean_rigid(36), retry_rigid(36);
  auto loads = old::Loads(f);
  const auto rows = f.cin_model.rows();
  ASSERT_GT(rows.count, 1u);
  const auto& last = rows.data[rows.count-1];
  const auto master = last.master_domain_nodes[0];
  const auto failing_node = last.secondary_domain_node;
  unsigned axis = 3;
  using namespace constraints::tied_shell;
  PatchInput geometry;
  geometry.secondary_position = cin::detail::ReadXyz(f.x.data(), failing_node);
  for (unsigned slot = 0; slot < 4; ++slot)
    geometry.master_position[slot] = cin::detail::ReadXyz(f.x.data(), last.master_domain_nodes[slot]);
  Patch patch;
  ASSERT_EQ(PreparePatch(geometry, patch), Status::Success);
  for (unsigned candidate = 0; candidate < 3 && axis == 3; ++candidate) {
    MasterMotion motion;
    for (unsigned slot = 0; slot < 4; ++slot) {
      if (last.master_domain_nodes[slot] != master) continue;
      if (candidate == 0) motion.velocity[slot].x = 1;
      if (candidate == 1) motion.velocity[slot].y = 1;
      if (candidate == 2) motion.velocity[slot].z = 1;
    }
    SecondaryMotion result;
    ASSERT_EQ(RecoverMotion(patch, motion, result), Status::Success);
    const auto spin = result.angular_velocity;
    if (std::fabs(spin.x)+std::fabs(spin.y)+std::fabs(spin.z) > .01) axis = candidate;
  }
  ASSERT_LT(axis, 3u);
  for (unsigned epoch=0; epoch<2; ++epoch) {
    SCOPED_TRACE(epoch);
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
    EXPECT_EQ(receipt.kick_dt, epoch==0 ? old::H/2 : old::H);
    owner.Discard();
    const auto original_force = loads[axis*n+master];
    loads[axis*n+master] = 1e16;
    old::Begin(owner, f, loads, true, token, assembly);
    const auto failure = old::Advance(owner, token, assembly, true);
    EXPECT_EQ(failure.status, NodalStatus::StepTooLarge);
    EXPECT_EQ(failure.node, failing_node);
    const auto previous_nodes = retry.nodes;
    receipt.owner_id = 993;
    EXPECT_NE(owner.CopyPrepared(token, retry.Nodes(), &receipt).status, NodalStatus::Ok);
    EXPECT_EQ(retry.nodes, previous_nodes);
    EXPECT_EQ(receipt.owner_id, 993u);
    fields::Accepted(owner, retry, after_stamp);
    after_groups.Read(owner);
    fields::Same(before, retry);
    old::Same(before_groups, after_groups);
    EXPECT_TRUE(trial_identity::SameStamp(before_stamp, after_stamp));
    loads[axis*n+master] = original_force;
    old::Begin(owner, f, loads, true, token, assembly);
    ASSERT_EQ(old::Advance(owner, token, assembly, true).status, NodalStatus::Ok);
    Prepared(owner, token, retry, retry_rigid, receipt);
    Capture(owner, token, retry_capture);
    fields::Same(clean, retry);
    SameDoubles(clean_rigid, retry_rigid);
    SameDoubles(clean_capture, retry_capture);
    old::Commit(owner, token, assembly);
    fields::Accepted(owner, retry, after_stamp);
    after_groups.Read(owner);
    for (unsigned g=0; g<after_groups.groups.size(); ++g)
      rigid::WriteGroupState(retry_rigid.data()+18*g, after_groups.groups[g].state);
    fields::Same(clean, retry);
    SameDoubles(clean_rigid, retry_rigid);
    EXPECT_EQ(after_stamp.epoch, epoch+1);
    EXPECT_EQ(owner.allocations().device_bytes, allocations.device_bytes);
    EXPECT_EQ(owner.allocations().device_allocations, allocations.device_allocations);
  }
}
} // namespace tl::fea::cin_parallel_test
