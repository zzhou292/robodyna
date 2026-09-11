// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "QephLayeredForceCore.h"
#include "lib_src/elements/sections/ShellLayeredJ2FailureForceAdapter.h"

namespace tl::fea::qeph {
struct LayeredJ2FailureHistory { History shell{};sections::ShellLayeredJ2FailureHistory section{}; };
struct LayeredJ2FailureForceTrial { ForceTrial force{};sections::ShellLayeredJ2FailureResult section{}; };
TL_QEPH_HD inline Status InitializeLayeredJ2FailureHistory(const ReferenceData& r,
    const sections::PointParameters& parameters,const sections::ConstantFailureParameters& failure,
    HistoryStamp stamp,LayeredJ2FailureHistory& output) noexcept {
  if(!sections::MatchesLayeredJ2Material(parameters,r.input)||!sections::ValidLayeredJ2Parameters(parameters)||
      !tl::math::Finite(failure.failure_strain)||!(failure.failure_strain>0)) return Status::kInvalidInput;
  LayeredJ2FailureHistory candidate;
  const auto status=InitializeHistory(r,stamp,candidate.shell);
  if(status!=Status::kSuccess) return status;
  output=candidate;return Status::kSuccess;
}
// Accepted OFF0/1 only, local centered NIP3 constant-D1. Saved point stresses
// and current force stresses are separate. OFF0 still evaluates nondegenerate
// geometry, rates and native history updates; no mass/contact/owner admission.
TL_QEPH_HD inline Status EvaluateLayeredJ2FailureForce(const ReferenceData& r,
    const sections::PointParameters& parameters,const sections::ConstantFailureParameters& failure,
    const LayeredJ2FailureHistory& accepted,const PrescribedInterval& interval,
    LayeredJ2FailureForceTrial& output) noexcept {
  return detail::EvaluateLayeredForce(r,parameters,accepted,interval,output,
      sections::LayeredJ2FailureForceAdapter{parameters,accepted.section,failure});
}
} // namespace tl::fea::qeph
