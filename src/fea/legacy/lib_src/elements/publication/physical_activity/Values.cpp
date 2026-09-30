// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../../../solvers/NodalNativePhysicalCoefficients.h"
namespace tl::fea::physical_activity {
PhysicalActivityReport PublicationReport(const ShellPublicationReport& r) noexcept {
  PhysicalActivityReport result;
  if (r.status != ShellPublicationStatus::Success) {
    result.status = PhysicalActivityStatus::PublicationFailure;
    result.message = r.message; result.publication_status = r.status; result.owner_status = r.nodal_status;
  }
  return result;
}
PhysicalActivityReport State::Sources() const noexcept {
  if (!usable) return {PhysicalActivityStatus::DeviceFailure, "Activity snapshot storage is poisoned"};
  return PublicationReport(publication->ValidatePhysicalSources(*owner, physical, participants, identity));
}
bool State::AdvanceGeneration() noexcept {
  if (phase == Phase::Exhausted || generation == UINT64_MAX) { phase = Phase::Exhausted; return false; }
  ++generation; return true;
}
void State::Invalidate() noexcept {
  if (AdvanceGeneration()) phase = Phase::Idle;
}
bool KinematicsOutputDisjoint(const DeviceNodalKinematicsView& v, const void* output, std::size_t bytes) noexcept {
  const auto range = [&](const void* pointer, std::size_t components) {
    return !pointer || (v.node_count <= SIZE_MAX / components / sizeof(double) &&
        trial_identity::Disjoint(output, bytes, pointer, v.node_count * components * sizeof(double)));
  };
  return range(v.position_xyz, 3) && range(v.velocity_xyz, 3) &&
      range(v.angular_velocity_xyz, 3) && range(v.orientation_wxyz, 4);
}
bool State::OutputDisjoint(const void* output, std::size_t bytes) const noexcept {
  using trial_identity::Disjoint;
  if (!output || !publication->PhysicalOutputDisjoint(output, bytes) ||
      !Disjoint(output, bytes, this, sizeof(*this)) ||
      !Disjoint(output, bytes, arena, layout.bytes)) return false;
  return KinematicsOutputDisjoint(assembly.accepted, output, bytes) &&
      KinematicsOutputDisjoint(prepared.kinematics, output, bytes) &&
      KinematicsOutputDisjoint(prepared.base_kinematics, output, bytes);

}
bool State::Live(Phase expected) const noexcept {
  if (phase != expected || !usable || !owner || !trial_identity::SameStamp(stamp, owner->accepted()) ||
      Sources().status != PhysicalActivityStatus::Ok) return false;
  return expected == Phase::Accepted
      ? publication->ValidatePhysicalAssembly(*owner, token, assembly).status == ShellPublicationStatus::Success
      : publication->ValidatePhysicalCandidate(*owner, token, diagnostics, prepared).status == ShellPublicationStatus::Success;
}
bool State::Authenticates(const PhysicalAcceptedActivityReceipt& r) const noexcept {
  return r.state_.lock().get() == this && r.generation_ == generation && Live(Phase::Accepted);
}
bool State::Authenticates(const PhysicalPreparedActivityReceipt& r) const noexcept {
  return r.state_.lock().get() == this && r.generation_ == generation && Live(Phase::Prepared);
}
PhysicalActivityDeviceView State::View() const noexcept {
  PhysicalActivityDeviceView view;
  const auto* endpoint = phase == Phase::Prepared ? current : base;
  if (forecast.qeph_count) view.qeph = {base, endpoint, summaries[0]};
  if (forecast.t3_count) view.t3 = {base + forecast.qeph_count, endpoint + forecast.qeph_count, summaries[1]};
  view.qbat_count = forecast.qbat_count;
  view.type45_count = diagnostics.has_type45 ? diagnostics.type45.joint_count : 0;
  view.accepted = stamp; view.attempt = attempt; view.generation = generation; view.stream = stream;
  return view;
}
} // namespace tl::fea::physical_activity
namespace tl::fea {
bool PhysicalAcceptedActivityReceipt::valid() const noexcept {
  const auto state = state_.lock(); return state && state->Authenticates(*this);
}
bool PhysicalPreparedActivityReceipt::valid() const noexcept {
  const auto state = state_.lock(); return state && state->Authenticates(*this);
}
} // namespace tl::fea
