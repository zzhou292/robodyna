// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected OpenRadioss layered material composition, (C) 2026 Siemens.
#pragma once
#include "T3LayeredForceCore.h"

namespace tl::fea::t3 {
// Value wrappers only: native History/ForceTrial layouts and LAW1 stay intact.
// Caller publishes the ordinary shell history and section sidecar together.
// The immutable prepared material/curve is a caller binding, not owned here.
struct LayeredJ2History { History shell{}; sections::ShellLayeredJ2History section{}; };
struct LayeredJ2ForceTrial {
  ForceTrial force{};
  sections::ShellLayeredJ2History proposed_section{};
  sections::ShellLayeredJ2Diagnostics section_diagnostics{};
};
TL_T3_HD inline Status InitializeLayeredJ2History(const ReferenceData& r,
    const sections::PointParameters& p,HistoryStamp stamp,LayeredJ2History& output) noexcept {
  if(r.input.placement!=ShellReferencePlacement::Centered) return Status::kInvalidInput;
  if(!sections::MatchesLayeredJ2Material(p,r.input)) return Status::kInvalidInput;
  LayeredJ2History candidate;
  const auto status=InitializeHistory(r,stamp,candidate.shell);
  if(status!=Status::kSuccess) return status;
  if(!sections::ValidLayeredJ2Parameters(p)) return Status::kInvalidInput;
  output=candidate; return Status::kSuccess;
}
// The optional source rate law consumes the same native total-rate scalar as
// the family diagnostic, including its accepted reported-thickness dependency.
TL_T3_HD inline Status EvaluateLayeredJ2Force(const ReferenceData& r,const sections::PointParameters& parameters,
    const LayeredJ2History& accepted,const PrescribedInterval& in,LayeredJ2ForceTrial& output) noexcept {
  if(r.input.placement!=ShellReferencePlacement::Centered) return Status::kInvalidInput;
  return detail::EvaluateLayeredForce(r,parameters,accepted,in,output,
      sections::LayeredJ2ForceAdapter{parameters,accepted.section});
}
} // namespace tl::fea::t3
