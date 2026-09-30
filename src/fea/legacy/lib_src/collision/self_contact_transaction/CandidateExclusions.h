// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "PolicyExclusions.h"

namespace tlfea::contact::self_contact_transaction {

// Exact native source context for one candidate pair. Preparation is deferred
// until continuous local/accepted-owner coverage has failed within its cap.
struct CandidateExclusions {
  const SelfContactActiveUseBinding& active_use;
  const CurrentFixedTriangle& first_accepted;
  const CurrentFixedTriangle& second_accepted;
  FixedTriangleFeatureTaskMask mask;
  const FixedContactFacet* descriptors;
  const std::uint32_t* triangle_order;
  std::size_t facet_count;
  SelfContactActivityView activity;
  std::size_t pair_ordinal;

  PolicyExclusionSource source() noexcept;

 private:
  static SelfContactTransactionReport Prepare(
      void*, AcceptedFeatureExclusionCertificate*, std::size_t,
      std::size_t*) noexcept;
};

}  // namespace tlfea::contact::self_contact_transaction
