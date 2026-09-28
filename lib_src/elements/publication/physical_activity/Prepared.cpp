// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include <utility>
namespace tl::fea {
PhysicalActivityReport PhysicalActivitySnapshot::CapturePrepared(FENodalState& owner,
    const NodalTrialToken& token, const ShellPhysicalDiagnostics& diagnostics,
    const NodalPreparedView& prepared, const PhysicalAcceptedActivityReceipt& accepted,
    PhysicalPreparedActivityReceipt* output) noexcept {
  namespace a = physical_activity; using S = PhysicalActivityStatus;
  if (!state_) return {S::NotInitialized, "Physical activity is not initialized"};
  auto& s = *state_;
  const auto fail = [&](PhysicalActivityReport r) { s.Invalidate(); return r; };
  using trial_identity::Disjoint;
  if (&owner != s.owner || accepted.state_.lock().get() != &s ||
      accepted.generation_ != s.generation || s.phase != a::Phase::Accepted ||
      !trial_identity::SameStamp(s.stamp, owner.accepted()) || s.attempt != prepared.attempt ||
      !s.OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !Disjoint(output, sizeof(*output), &token, sizeof(token)) ||
      !Disjoint(output, sizeof(*output), &diagnostics, sizeof(diagnostics)) ||
      !Disjoint(output, sizeof(*output), &prepared, sizeof(prepared)) ||
      !Disjoint(output, sizeof(*output), &accepted, sizeof(accepted)))
    return fail({S::StaleReceipt, "Prepared activity inputs or accepted receipt are stale or overlapping"});
  if (s.generation == UINT64_MAX) return fail({S::ResourceLimit, "Physical activity generation space is exhausted"});
  auto r = a::PublicationReport(s.publication->ValidatePhysicalCandidate(owner, token, diagnostics, prepared));
  if (r.status != S::Ok) return fail(r);
  if (!a::KinematicsOutputDisjoint(prepared.kinematics, output, sizeof(*output)) ||
      !a::KinematicsOutputDisjoint(prepared.base_kinematics, output, sizeof(*output)))
    return fail({S::InvalidInput, "Prepared activity output overlaps nodal arrays"});
  r = s.Sources(); if (r.status != S::Ok) return fail(r);
  r = s.Capture(token, nullptr, &prepared, diagnostics);
  if (r.status != S::Ok) return fail(r);
  s.AdvanceGeneration(); std::swap(s.current, s.staging);
  s.token = token; s.prepared = prepared; s.diagnostics = diagnostics; s.phase = a::Phase::Prepared;
  PhysicalPreparedActivityReceipt next; next.state_ = state_; next.generation_ = s.generation;
  *output = std::move(next); return {};
}
PhysicalActivityReport PhysicalActivitySnapshot::BorrowPrepared(FENodalState& owner,
    const NodalTrialToken& token, const ShellPhysicalDiagnostics& diagnostics,
    const NodalPreparedView& prepared, const PhysicalPreparedActivityReceipt& receipt,
    PhysicalActivityDeviceView* output) const noexcept {
  using S = PhysicalActivityStatus; using trial_identity::Disjoint;
  if (!state_) return {S::NotInitialized, "Physical activity is not initialized"};
  const auto& s = *state_;
  if (&owner != s.owner || !s.Authenticates(receipt) ||
      !trial_identity::SamePrepared(prepared, s.prepared))
    return {S::StaleReceipt, "Prepared activity receipt is stale or belongs to another attempt"};
  const auto r = physical_activity::PublicationReport(
      s.publication->ValidatePhysicalCandidate(owner, token, diagnostics, prepared));
  if (r.status != S::Ok) return r;
  if (!s.OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !Disjoint(output, sizeof(*output), &token, sizeof(token)) ||
      !Disjoint(output, sizeof(*output), &diagnostics, sizeof(diagnostics)) ||
      !Disjoint(output, sizeof(*output), &prepared, sizeof(prepared)) ||
      !Disjoint(output, sizeof(*output), &receipt, sizeof(receipt)))
    return {S::InvalidInput, "Activity view output overlaps source or receipt"};
  *output = s.View(); return {};
}
} // namespace tl::fea
