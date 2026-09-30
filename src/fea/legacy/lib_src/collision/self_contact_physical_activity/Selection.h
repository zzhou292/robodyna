// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../SelfContactPhysicalActivityTypes.h"

namespace tlfea::contact::self_contact_physical_activity {

// Borrowed immutable rows only. source_at returns a source row, and exact_row
// authenticates its supported family, in-range index and nonzero source ID
// against the same physical inventory. Both callbacks must be nonthrowing.
// Keep validation in source order: a malformed row precedes its duplicate
// check, and the earliest offending row determines the complete diagnostic.
template<class SourceAt, class ExactRow>
SelfContactPhysicalActivityReport ValidateSelectionRows(
    std::size_t count, SourceAt source_at, ExactRow exact_row) noexcept {
  using Status = SelfContactPhysicalActivityStatus;
  if (!count) {
    SelfContactPhysicalActivityReport report;
    report.status = Status::InvalidInput;
    report.message = "Self-contact activity selection is empty";
    return report;
  }

  bool increasing_prefix = true;
  std::uint64_t previous_source_id = 0;
  for (std::size_t parent = 0; parent < count; ++parent) {
    const auto& source = source_at(parent);
    if (!exact_row(source)) {
      SelfContactPhysicalActivityReport report;
      report.status = Status::IdentityMismatch;
      report.message =
          "Selected parent is not an exact QEPH/T3/QBAT inventory row";
      report.parent = parent;
      report.family = source.family;
      report.family_index = source.family_index;
      return report;
    }

    // An exact physical family/index has one source ID. A strictly increasing
    // prefix therefore cannot repeat that family/index. Prove ordering on this
    // call rather than trusting how the binding was built. After any ordering
    // failure retain the original scan for this row and every following row.
    increasing_prefix = increasing_prefix &&
        previous_source_id < source.source_parent_id;
    previous_source_id = source.source_parent_id;
    if (increasing_prefix) continue;

    for (std::size_t prior = 0; prior < parent; ++prior) {
      const auto& other = source_at(prior);
      if (other.family == source.family &&
          other.family_index == source.family_index) {
        SelfContactPhysicalActivityReport report;
        report.status = Status::IdentityMismatch;
        report.message =
            "Selected activity inventory repeats a physical family row";
        report.parent = parent;
        report.family = source.family;
        report.family_index = source.family_index;
        return report;
      }
    }
  }
  return {};
}

}  // namespace tlfea::contact::self_contact_physical_activity
