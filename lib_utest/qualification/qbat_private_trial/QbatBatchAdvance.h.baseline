// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "QbatBatchArena.h"
#include "QbatForce.h"

namespace tl::fea::qbat::batch_detail {
TL_QBAT_HD inline BatchResult PackResult(const ForceTrial& force) noexcept {
  BatchResult result;
  result.history=force.proposed_history.data();
  result.stamp=force.proposed_history.stamp();
  result.kinematics=force.kinematics;
  for(unsigned point=0;point<4;++point) {
    result.point[point]=force.point[point];
    result.internal_force_n[point]=force.internal_force_n[point];
    result.internal_couple_nm[point]=force.internal_couple_nm[point];
  }
  result.diagnostics=force.diagnostics;
  return result;
}

TL_QBAT_HD inline Status InitializeResult(const Element& element,BatchResult& output) noexcept {
  History virgin;
  const auto status=InitializeHistory(element.reference,element.material,element.failure,{0,0},virgin);
  if(status!=Status::kSuccess) return status;
  BatchResult result;
  result.history=virgin.data();
  result.stamp=virgin.stamp();
  output=result;
  return Status::kSuccess;
}

// Only the immutable model contributes pointers to this transient pure packet.
// The accepted and proposed resident rows contain numerical values exclusively.
TL_QBAT_HD inline Status Advance(const Element& element,const BatchResult& accepted,
                                const PrescribedInterval& interval,BatchResult& output) noexcept {
  History base;
  auto status=PreparePrescribedHistory(element.reference,element.material,element.failure,
      accepted.history,accepted.stamp,base);
  if(status!=Status::kSuccess) return status;
  ForceTrial proposed;
  status=EvaluateForce(element.reference,element.material,element.failure,base,interval,proposed);
  if(status!=Status::kSuccess) return status;
  output=PackResult(proposed);
  return Status::kSuccess;
}
} // namespace tl::fea::qbat::batch_detail
