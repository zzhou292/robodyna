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
    const auto checked = batch.PreflightPublication(owner, token, prepared, expected);
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
