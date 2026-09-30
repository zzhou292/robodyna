// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/type13/resident/Storage.h"
#include "lib_src/solvers/ExplicitNodalStep.h"

namespace tl::fea::type13 {
// Qualification only. The production common coordinator has not yet admitted
// mapped TYPE13 participants. This peer validates one closed test contributor,
// commits the sole real owner once, then performs the infallible slab switch.
class BatchQualificationPeer {
 public:
  // Opaque test-only claimant identities, never dereferenced or instantiated as
  // production coordinators. The real coordinator owns its own object address.
  static const ShellBatchPublication* Scope(unsigned index = 0) {
    static unsigned char tags[2]{};
    return reinterpret_cast<const ShellBatchPublication*>(&tags[index]);
  }
  static BatchReport PreflightAttach(Batch& batch, const BatchConfig& config,
      const Type13NodeContributions& source, const ShellBatchPublication* claimant) {
    return batch.PreflightAttach(config.owner, source, config.configuration_id,
        config.qualification_id, config.startup, config.assembly, claimant);
  }
  static void Attach(Batch& batch, const ShellBatchPublication* claimant) {
    batch.AttachPublication(claimant);
  }
  static void Release(Batch& batch, const ShellBatchPublication* claimant) {
    batch.ReleasePublication(claimant);
  }
  static void Poison(Batch& batch) { batch.Poison(); }
  static BatchReport Preflight(Batch& batch, FENodalState& owner,
      const NodalTrialToken& token, const NodalPreparedView& prepared,
      const BatchDiagnostics& expected, const ShellBatchPublication* claimant) {
    return batch.PreflightPublication(owner, token, prepared, expected, claimant);
  }
  static BatchReport Commit(Batch& batch, FENodalState& owner,
                             const NodalTrialToken& token,
                             const NodalPreparedView& prepared,
                             const BatchDiagnostics& expected,
                             bool other_checks_passed = true) {
    auto discard = [&](BatchReport report) {
      owner.Discard();
      batch.DiscardTrial();
      return report;
    };
    // Existing closed one-contributor oracle fixtures bypass attachment setup
    // only through this test friend. PublicationScopeTest exercises the real
    // initial preflight, duplicate ownership and claimant checks separately.
    if (!batch.impl_->publication_scope) batch.AttachPublication(Scope());
    const auto checked = batch.PreflightPublication(owner, token, prepared, expected, Scope());
    if (!checked) {
      return discard(checked);
    }
    if (!other_checks_passed) {
      return discard({BatchStatus::ElementFailure, "Test-only later contributor rejected"});
    }
    auto report = CompleteNodalValidation(owner, token,
        {prepared.owner_id, expected.base_epoch, prepared.attempt, expected.qualification_id, true});
    if (report.status != NodalStatus::Ok) {
      return discard({BatchStatus::NodalFailure, report.message});
    }
    report = owner.Commit(token);
    if (report.status != NodalStatus::Ok) {
      return discard({BatchStatus::NodalFailure, report.message});
    }
    batch.Publish(owner.accepted());
    return {};
  }
  // Explicit fault injection into qualification-owned storage. Never a public
  // runtime mutator; tests restore the field before continuing a trajectory.
  static Evaluation* AcceptedDeviceResults(Batch& batch) {
    return batch.impl_->device_header.slab[batch.impl_->accepted_slab];
  }
};
} // namespace tl::fea::type13
