// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../../type45/resident/Storage.h"
namespace tl::fea::physical_activity {
PhysicalActivityReport BatchAccess::Type45(const type45::Batch* batch,
    const type45::Model* expected, bool check_source, FENodalState& owner,
    ShellBatchPublication& publication, const ShellPhysicalBinding& physical,
    bool has_diagnostics, const type45::BatchDiagnostics& diagnostics) noexcept {
  using S = PhysicalActivityStatus; using F = PhysicalActivityFamily;
  const auto mismatch = [](const char* message) {
    return PhysicalActivityReport{S::SourceMismatch, message, F::Type45};
  };
  if (bool(batch) != has_diagnostics || (check_source && bool(batch) != bool(expected)))
    return mismatch("TYPE45 source, participant and diagnostic presence differ");
  if (!batch) return {};
  if (!batch->impl_) return mismatch("TYPE45 participant is not initialized");
  const auto& s = *batch->impl_;
  if (!physical.prepared() || !physical.domain() || !physical.coefficients() ||
      !s.usable || !s.bound || s.publication_scope != &publication ||
      !s.physical_scope || !s.physical_scope->Matches(physical) ||
      !trial_identity::SameStamp(s.accepted_stamp, owner.accepted()) ||
      !s.model.prepared() || !s.model.domain() || !s.model.rigid_binding() ||
      !s.model.rigid_binding()->coefficients() ||
      !s.model.domain()->SharesStorage(*physical.domain()) ||
      !s.model.rigid_binding()->coefficients()->Matches(*physical.coefficients()) ||
      (check_source && !s.model.SharesStorage(*expected)))
    return mismatch("TYPE45 model is not the actual bound physical participant source");
  if (!diagnostics.valid || diagnostics.joint_count != s.model.joints().size() ||
      diagnostics.source_instance_id != s.model.source_instance_id())
    return mismatch("TYPE45 complete joint count or source identity differs");
  const auto rigid = owner.ValidateRigidAssemblyBinding(*s.model.rigid_binding());
  if (rigid.status != NodalStatus::Ok) return mismatch(rigid.message);
  // The admitted TYPE45 operator has no parent-off transition. This proves
  // the complete always-active joint roster, not a new removal capability.
  return {};
}
} // namespace tl::fea::physical_activity
namespace tl::fea {
PhysicalActivityReport PhysicalActivitySnapshot::ValidateType45Source(const type45::Model* model) const noexcept {
  using S = PhysicalActivityStatus;
  if (!state_) return {S::NotInitialized, "Physical activity is not initialized"};
  auto& s = *state_;
  auto r = s.Sources(); if (r.status != S::Ok) return r;
  ShellPhysicalDiagnostics diagnostics;
  r = physical_activity::PublicationReport(
      s.publication->CopyAcceptedPhysicalDiagnostics(s.owner->accepted(), &diagnostics));
  if (r.status != S::Ok) return r;
  return physical_activity::BatchAccess::Type45(s.participants.type45, model, true,
      *s.owner, *s.publication, s.physical, diagnostics.has_type45, diagnostics.type45);
}
} // namespace tl::fea
