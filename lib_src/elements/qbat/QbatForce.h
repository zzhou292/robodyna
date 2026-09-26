// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected four-point CBAFORC3 recurrence: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "QbatForceBody.h"
namespace tl::fea::qbat {
// All public output fields, including aliased accepted history, survive failure.
TL_QBAT_HD inline Status EvaluateForce(const Reference& reference,const Material& material,
    Failure failure,const History& accepted,const PrescribedInterval& interval,ForceTrial& output) noexcept {
  if (!accepted.prepared() || !detail::ValidMaterial(reference,material,failure) ||
      !detail::SameReference(reference,accepted.reference()) ||
      !detail::SameMaterial(material,accepted.material()) ||
      !detail::Same(failure.failure_strain,accepted.failure().failure_strain)) return Status::kInvalidReference;
  ForceTrial trial;
  HistoryValues proposed;
  HistoryStamp stamp;
  auto status = detail::EvaluateForceBody(reference,material,failure,accepted.data(),accepted.stamp(),interval,
      {proposed,stamp,trial.kinematics,trial.point,trial.internal_force_n,trial.internal_couple_nm,trial.diagnostics});
  if (status != Status::kSuccess) return status;
  status = PreparePrescribedHistory(reference,material,failure,proposed,stamp,trial.proposed_history);
  if (status != Status::kSuccess) return Status::kNonfiniteResult;
  output = trial;
  return Status::kSuccess;
}
} // namespace tl::fea::qbat
