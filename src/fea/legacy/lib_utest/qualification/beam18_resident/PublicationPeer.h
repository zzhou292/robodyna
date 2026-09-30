// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/beam18/resident/Storage.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
namespace tl::fea::beam18 {
// Qualification-only coordinator. The real owner commits once, after every
// contributor preflight; this peer never implements a second time integrator.
class BatchQualificationPeer {
 public:
  static const ShellBatchPublication* Scope(unsigned n = 0) {
    static unsigned char tags[2]{};
    return reinterpret_cast<const ShellBatchPublication*>(&tags[n]);
  }
  static BatchReport PreflightAttach(Batch& b, FENodalState& owner,
      const NodalCoefficientLedger& ledger, const NodalRigidAssemblyBinding& rigid,
      const NodalCinWitnessSource& cin, const Model& model, const BatchConfig& config,
      const ShellBatchPublication* claimant = Scope()) {
    return b.PreflightAttach(owner, ledger, rigid, cin, model, config, claimant);
  }
  static void Attach(Batch& b) { b.AttachPublication(Scope()); }
  static BatchReport Preflight(Batch& b, FENodalState& owner, const NodalTrialToken& token,
      const NodalPreparedView& view, const BatchDiagnostics& expected,
      const ShellBatchPublication* claimant = Scope()) {
    return b.PreflightPublication(owner, token, view, expected, claimant);
  }
  static BatchReport Commit(Batch& b, FENodalState& owner, const NodalTrialToken& token,
      const NodalPreparedView& view, const BatchDiagnostics& expected, bool other_ready = true) {
    auto result = Preflight(b, owner, token, view, expected);
    if (result && !other_ready) result = {BatchStatus::ElementFailure, "Later test contributor rejected"};
    if (result) {
      auto checked = CompleteNodalValidation(owner, token,
          {view.owner_id, expected.base_epoch, view.attempt, expected.qualification_id, true});
      if (checked.status == NodalStatus::Ok) checked = owner.Commit(token);
      if (checked.status != NodalStatus::Ok) result = {BatchStatus::NodalFailure, checked.message};
    }
    if (!result) { owner.Discard(); b.DiscardTrial(); return result; }
    b.Publish(owner.accepted());
    return {};
  }
  static BatchReport ReadConstructed(Batch& b, ResultBuffer output, BatchDiagnostics& diagnostics) {
    auto& state = *b.impl_;
    if (state.bound || !state.OutputBuffers(output, &diagnostics, sizeof(diagnostics), &b, sizeof(b)))
      return {BatchStatus::InvalidInput, "Constructor read requires unclaimed exact output"};
    const auto report = state.ReadResults(0, state.accepted_diagnostics);
    if (!report) return report;
    state.PublishResults(output);
    diagnostics = state.accepted_diagnostics;
    return {};
  }
  static double* LastPreparedCouple(Batch& b) {
    return &b.impl_->device_header.slab[b.impl_->TrialSlab()][b.impl_->model.parents().size()-1].rhs_couple_nm[1].z;
  }
};
} // namespace tl::fea::beam18
