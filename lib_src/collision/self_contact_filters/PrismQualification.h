// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../SelfContactFilterCertificates.h"

namespace tlfea::contact::self_contact_filters {
// Per-call qualification counts only: at most 148 axis attempts and 36
// three-dimensional point comparisons (not individual scalar comparisons).
// They are not physical proof work, runtime profiles or contact authority.
struct PrismCounts {
  unsigned axis_tests = 0, coordinate_tests = 0;
  bool hull_coincidence = false;
};
struct PrismObservation {
  bool separated = false, valid = false;
  SelfContactFacetPrismSeparationAxis axis = SelfContactFacetPrismSeparationAxis::None;
  PrismCounts counts;
};
struct PrismComparison { PrismObservation original, current; };
PrismComparison ComparePrismHullCoincidence(
    const CurrentFixedTriangle&, const CurrentFixedTriangle&, double,
    const CurrentFixedTriangle&, const CurrentFixedTriangle&, double,
    SelfContactFacetPrismAxisLimit) noexcept;
}  // namespace tlfea::contact::self_contact_filters
