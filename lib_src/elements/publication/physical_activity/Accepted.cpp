// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include <utility>
namespace tl::fea {
PhysicalActivityReport PhysicalActivitySnapshot::CaptureAccepted(FENodalState& owner,
    const NodalTrialToken& token, const NodalAssemblyView& assembly,
    PhysicalAcceptedActivityReceipt* output) noexcept {
  namespace a = physical_activity; using S = PhysicalActivityStatus;
  if (!state_) return {S::NotInitialized, "Physical activity is not initialized"};
  auto& s = *state_;
  const auto fail = [&](PhysicalActivityReport r) { s.Invalidate(); return r; };
  using trial_identity::Disjoint;
  if (&owner != s.owner || !s.OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !Disjoint(output, sizeof(*output), &token, sizeof(token)) ||
      !Disjoint(output, sizeof(*output), &assembly, sizeof(assembly)) ||
      !owner.AssemblyRangeDisjoint(token, assembly, output, sizeof(*output)))
    return fail({S::InvalidInput, "Activity accepted owner/view/output is invalid or overlapping"});
  if (s.phase == a::Phase::Exhausted || s.generation == UINT64_MAX)
    return fail({S::ResourceLimit, "Physical activity generation space is exhausted"});
  if (s.attempt == assembly.attempt &&
      s.stamp.owner_id == assembly.owner_id && s.stamp.epoch == assembly.accepted.base_epoch)
    return fail({S::StaleReceipt, "Physical activity attempt was already captured"});
  auto r = a::PublicationReport(s.publication->ValidatePhysicalAssembly(owner, token, assembly));
  if (r.status != S::Ok) return fail(r);
  r = s.Sources(); if (r.status != S::Ok) return fail(r);
  ShellPhysicalDiagnostics diagnostics;
  r = a::PublicationReport(s.publication->CopyAcceptedPhysicalDiagnostics(owner.accepted(), &diagnostics));
  if (r.status != S::Ok) return fail(r);
  r = s.Capture(token, &assembly, nullptr, diagnostics);
  if (r.status != S::Ok) return fail(r);
  s.AdvanceGeneration(); std::swap(s.base, s.staging);
  s.token = token; s.assembly = assembly; s.prepared = {}; s.diagnostics = diagnostics;
  s.stamp = owner.accepted(); s.attempt = assembly.attempt; s.phase = a::Phase::Accepted;
  PhysicalAcceptedActivityReceipt next; next.state_ = state_; next.generation_ = s.generation;
  *output = std::move(next); return {};
}
PhysicalActivityReport PhysicalActivitySnapshot::BorrowAccepted(FENodalState& owner,
    const NodalTrialToken& token, const NodalAssemblyView& assembly,
    const PhysicalAcceptedActivityReceipt& receipt, PhysicalActivityDeviceView* output) const noexcept {
  using S = PhysicalActivityStatus; using trial_identity::Disjoint;
  if (!state_) return {S::NotInitialized, "Physical activity is not initialized"};
  const auto& s = *state_;
  if (&owner != s.owner || !s.Authenticates(receipt) ||
      !trial_identity::SameAssembly(assembly, s.assembly))
    return {S::StaleReceipt, "Accepted activity receipt is stale or belongs to another attempt"};
  const auto r = physical_activity::PublicationReport(s.publication->ValidatePhysicalAssembly(owner, token, assembly));
  if (r.status != S::Ok) return r;
  if (!s.OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !Disjoint(output, sizeof(*output), &token, sizeof(token)) ||
      !Disjoint(output, sizeof(*output), &assembly, sizeof(assembly)) ||
      !Disjoint(output, sizeof(*output), &receipt, sizeof(receipt)) ||
      !owner.AssemblyRangeDisjoint(token, assembly, output, sizeof(*output)))
    return {S::InvalidInput, "Activity view output overlaps source or receipt"};
  *output = s.View(); return {};
}
} // namespace tl::fea
