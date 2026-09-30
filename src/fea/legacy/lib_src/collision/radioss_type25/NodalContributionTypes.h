// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "CoefficientTypes.h"
#include <cstdint>

namespace tlfea::contact::radioss_type25 {

enum class SolidNodalKind { Unspecified, Hex8, Penta6 };
struct NativeSolidNodalInput {
  // Actual native NNC, not the number of unique connectivity node IDs.
  SolidNodalKind kind = SolidNodalKind::Unspecified;
  double volume = 0, fill = 1, bulk = 0; // Prepared VOLU, FILL and PM32.
};
struct NativeSolidNodalShares {
  // Volume and pressure*volume per defined raw occurrence, in native units.
  double volume_share = 0, bulk_volume_share = 0;
  // Bit0 is raw slot1. Penta6 leaves raw slots4 and8 undefined.
  std::uint8_t defined_raw_slot_mask = 0;
};

enum class SpringNodalKind { Unspecified, Type13, Type25 };
struct NativeResolvedSlopeScale {
  double slope = 0, scale = 1;
};
struct NativeSpringNodalInput {
  SpringNodalKind kind = SpringNodalKind::Unspecified;
  // Actual I7STIFS. Only its nonzero branch defines this arithmetic substage.
  int interface_initialization = 0;
  // Actual ILENG. Nonpositive selects ONE without reading geometric_length.
  int length_mode = 0;
  // Resolved GEO3/41,10/45,15/49; the third channel is unread for Type25.
  NativeResolvedSlopeScale translation[3]{};
  // Authentic prepared native length, consumed only when ILENG>0.
  double geometric_length = 0;
};

// Native arithmetic only: no material/geometry/contributor authority is supplied.
// Spring slope dimensions depend on the resolved length mode/property; no
// universal SI slope conversion is implied by these native packet values.
} // namespace tlfea::contact::radioss_type25
