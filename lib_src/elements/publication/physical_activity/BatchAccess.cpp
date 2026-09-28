// SPDX-License-Identifier: AGPL-3.0-or-later
#include "BatchAccess.h"
#include "../../qeph/QephBatchStorage.h"
#include "../../t3/T3BatchStorage.h"
#include "../../../solvers/NodalNativePhysicalCoefficients.h"
namespace tl::fea::physical_activity {
namespace {
template<class State, class Diagnostics, class Input, class Same>
PhysicalActivityReport BorrowFamily(State& s, FENodalState& owner,
    ShellBatchPublication& publication, const ShellPhysicalBinding& physical,
    const NodalTrialToken& token, const NodalAssemblyView* assembly,
    const NodalPreparedView* prepared, const Diagnostics& expected, Input& output, Same same) {
  using S = PhysicalActivityStatus;
  if (!s.usable || !s.bound || !s.physical || !s.physical->Matches(physical) ||
      s.publication_scope != &publication || !s.joined_binding || !s.plasticity ||
      !s.storage || !s.plasticity->mixed_device() || !s.plasticity->failure_device() ||
      !trial_identity::SameStamp(s.accepted_stamp, owner.accepted()))
    return {S::SourceMismatch, "Activity batch is not this live mapped physical participant"};
  if (bool(assembly) == bool(prepared))
    return {S::InvalidInput, "Activity requires exactly one endpoint phase"};
  const auto& actual = prepared ? s.candidate_diagnostics : s.accepted_diagnostics;
  if (!same(actual, expected) || (prepared && !s.pending))
    return {S::StaleReceipt, "Activity family diagnostics are not the complete live endpoint"};
  const auto authenticated = prepared
      ? native_physical_coefficients::AuthenticatePrepared(owner, token, s.accepted_stamp, *prepared)
      : owner.AuthenticateAssemblyView(token, *assembly);
  if (authenticated.status != NodalStatus::Ok)
    return {S::OwnerFailure, authenticated.message};
  if ((prepared && !trial_identity::SamePrepared(s.candidate_view, *prepared)) ||
      (!prepared && (s.assembled_epoch != s.accepted_stamp.epoch ||
                    s.assembled_attempt != assembly->attempt)) ||
      s.stream != (prepared ? prepared->stream : assembly->stream))
    return {S::StaleReceipt, "Activity batch view, assembly or stream differs"};
  const auto slab = s.AcceptedSlabIndex() ^ (prepared ? 1u : 0u);
  const auto shape = s.plasticity->CheckActivityFailureSources(slab, s.config.element_count);
  if (shape.status != shell_batch_plasticity_detail::SetupStatus::Success)
    return {S::SourceMismatch, shape.message};
  const auto pending = s.PendingError();
  if (pending.status != decltype(pending.status)::Success)
    return {S::DeviceFailure, pending.message};
  Input next;
  next.elements = s.device_header.model.element;
  next.results = s.device_header.slab[slab].element;
  next.mixed = s.plasticity->mixed_device();
  next.failure = s.plasticity->failure_device();
  next.point = s.plasticity->one_point_device();
  next.count = s.config.element_count;
  next.slab = slab;
  next.time = expected.time;
  next.epoch = expected.epoch;
  next.force_time = s.accepted_stamp.time + (prepared ? s.config.owner.fixed_dt : 0);
  next.force_epoch = s.accepted_stamp.epoch + (prepared ? 1 : 0);
  next.execution = physical.catalog()->execution_sections();
  if (!next.elements || !next.results || !next.count)
    return {S::SourceMismatch, "Activity participant device operands are missing"};
  output = next;
  return {};
}
}
PhysicalActivityReport BatchAccess::Borrow(qeph::QephBatch& batch, FENodalState& owner,
    ShellBatchPublication& pub, const ShellPhysicalBinding& source, const NodalTrialToken& token,
    const NodalAssemblyView* a, const NodalPreparedView* p,
    const qeph::BatchDiagnostics& d, QephInput& output) {
  if (!batch.impl_) return {PhysicalActivityStatus::NotInitialized, "QEPH activity batch absent"};
  auto result = BorrowFamily(*batch.impl_, owner, pub, source, token, a, p, d, output,
      qeph::batch_detail::SameDiagnostics);
  result.family = PhysicalActivityFamily::Qeph;
  return result;
}
PhysicalActivityReport BatchAccess::Borrow(t3::T3Batch& batch, FENodalState& owner,
    ShellBatchPublication& pub, const ShellPhysicalBinding& source, const NodalTrialToken& token,
    const NodalAssemblyView* a, const NodalPreparedView* p,
    const t3::BatchDiagnostics& d, T3Input& output) {
  if (!batch.impl_) return {PhysicalActivityStatus::NotInitialized, "T3 activity batch absent"};
  auto result = BorrowFamily(*batch.impl_, owner, pub, source, token, a, p, d, output,
      t3::batch_detail::SameDiagnostics);
  result.family = PhysicalActivityFamily::T3;
  return result;
}
} // namespace tl::fea::physical_activity
