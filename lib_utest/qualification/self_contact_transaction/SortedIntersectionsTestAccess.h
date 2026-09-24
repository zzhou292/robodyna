// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/self_contact_transaction/SortedIntersections.h"

namespace tlfea::contact::self_contact_transaction {
// Qualification-only constructor access. Raw rows still pass the exact same
// canonical/strict-order scan as native publications; no trusted-sorted flag.
struct SortedIntersectionsTestAccess {
  static SortedIntersections FromRaw(FixedTriangleIntersectionView view) noexcept {
    return SortedIntersections(view);
  }
  static SortedIntersections FromPublisher(const FixedTriangleFeatureDiscovery& owner) noexcept {
    return SortedIntersections(owner);
  }
};
}  // namespace tlfea::contact::self_contact_transaction
