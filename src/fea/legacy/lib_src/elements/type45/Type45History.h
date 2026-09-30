// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type45Reference.h"

namespace tl::fea::type45 {
struct Evaluation;
class History {
 public:
  TL_TYPE45_HD const HistoryValues& values() const { return values_; }
  TL_TYPE45_HD Stamp stamp() const { return stamp_; }
  TL_TYPE45_HD bool ready() const { return reference_.ready(); }
  TL_TYPE45_HD const Reference& reference() const { return reference_; }
  TL_TYPE45_HD bool Matches(const Reference& reference) const { return reference_.Matches(reference); }
  // The only initial history is the native virgin TT0 state. This does not
  // evaluate force after auto-K or manufacture an owner acceptance receipt.
  TL_TYPE45_HD static Status Initialize(const Reference& reference, History& output) {
    if (!reference.ready()) return Status::ReferenceMismatch;
    History next;
    next.reference_=reference;
    next.values_.frame=reference.frame();
    output=next;
    return Status::Success;
  }
 private:
  Reference reference_{};
  Stamp stamp_{};
  HistoryValues values_{};
  friend TL_TYPE45_HD Status Evaluate(const Reference&,const History&,const Interval&,Evaluation&);
};
struct Evaluation {
  History history;
  EndpointResult endpoint[2]{};
  Diagnostics diagnostics;
};
} // namespace tl::fea::type45
