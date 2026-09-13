// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "SelfContactActiveUseBinding.h"
#include "SelfContactCurrentRegularityTypes.h"
#include "SelfContactCurrentRegularityValues.h"

#include <memory>

namespace tlfea::contact {

// Bounded host-only current geometry certificate for the fixed-facet profile.
// It retains the exact active-use authority, certifies every base-active
// parent (including 1->0 removal), and may explicitly skip only 0->0 parents.
// It adds no force, broadphase, crossing, clock, refinement or timestep state.
class SelfContactCurrentRegularity {
 public:
  SelfContactCurrentRegularity() noexcept;
  ~SelfContactCurrentRegularity();
  SelfContactCurrentRegularity(const SelfContactCurrentRegularity&) = delete;
  SelfContactCurrentRegularity& operator=(
      const SelfContactCurrentRegularity&) = delete;
  SelfContactCurrentRegularity(SelfContactCurrentRegularity&&) noexcept;
  SelfContactCurrentRegularity& operator=(
      SelfContactCurrentRegularity&&) noexcept;

  static SelfContactCurrentRegularityPreflight Preflight(
      const SelfContactActiveUseBinding&,
      SelfContactCurrentRegularityLimits = {}) noexcept;
  SelfContactCurrentRegularityReport Initialize(
      const SelfContactActiveUseBinding&,
      SelfContactCurrentRegularityLimits = {}) noexcept;
  SelfContactCurrentRegularityReport Certify(
      VectorView positions, SelfContactActivityView activity,
      SelfContactCurrentRegularityReceipt* receipt) noexcept;

  // Pure receipt-gated transformation.  Only the unresolved own-parent status
  // can become ExcludedRegularOwnParent; every other exclusion/admission is
  // left outside this module.
  SelfContactCurrentRegularityReport ExcludeCertifiedOwnParent(
      const SelfContactPairClassification&,
      const SelfContactCurrentRegularityReceipt&,
      SelfContactPairClassification* output) const noexcept;

  bool initialized() const noexcept { return bool(impl_); }
  const SelfContactActiveUseBinding* binding() const noexcept;
  SelfContactCurrentRegularityForecast forecast() const noexcept;
  SelfContactCurrentRegularityView results() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tlfea::contact
