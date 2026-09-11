// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "QephLayeredForceCore.h"
#include "lib_src/elements/sections/ShellLayeredTab1ForceAdapter.h"

namespace tl::fea::qeph {
struct LayeredTab1History {
  History shell{};
  sections::ShellLayeredTab1History section{};
};
struct LayeredTab1ForceTrial {
  ForceTrial force{};
  sections::ShellLayeredTab1Result section{};
};
TL_QEPH_HD inline Status InitializeLayeredTab1History(const ReferenceData& reference,
    const sections::PointParameters& material,const sections::ShellLayeredTab1Parameters& failure,
    HistoryStamp stamp,LayeredTab1History& output) noexcept {
  if(!sections::MatchesLayeredJ2Material(material,reference.input)||
      !sections::ValidLayeredTab1ForceParameters(material,failure)) return Status::kInvalidInput;
  LayeredTab1History candidate;
  const auto status=InitializeHistory(reference,stamp,candidate.shell);
  if(status!=Status::kSuccess) return status;
  output=candidate;
  return Status::kSuccess;
}

// Centered analytic LAW44/FilteredZeroC NIP3 and explicit TAB1 AnyPoint.
// OFF0 preserves native history updates while the existing core masks forces.
// No source-placement, resident collection, mass or contact admission is added.
TL_QEPH_HD inline Status EvaluateLayeredTab1Force(const ReferenceData& reference,
    const sections::PointParameters& material,const sections::ShellLayeredTab1Parameters& failure,
    const LayeredTab1History& accepted,const PrescribedInterval& interval,
    LayeredTab1ForceTrial& output) noexcept {
  return detail::EvaluateLayeredForce(reference,material,accepted,interval,output,
      sections::LayeredTab1ForceAdapter{material,accepted.section,failure});
}
} // namespace tl::fea::qeph
