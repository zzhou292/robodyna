// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../CoefficientTypes.h"
#include "../FrictionTypes.h"
namespace tlfea::contact::radioss_type25 {
// Raw geometry packet for an already-proved EightSlot support.
// Exterior and internal entries have separate source-defined result channels.
// It does not establish membership, X versus projected X_C, material ownership,
// topology or input source authenticity. All coordinates are native lengths.
struct NativeExteriorMainGeometryInput {
  ShellLayout layout = ShellLayout::Unspecified;
  Vector face[4]{};      // Primary AFTER SH2 and BEFORE I25GAPM/INSOL3D.
  Vector solid_raw[8]{}; // Reader IXS(2:9), including repeats, BEFORE INITIA.
};
struct NativeExteriorMainGeometryResult {
  Vector normal_before_orientation;
  double area = 0, signed_volume = 0, center_projection = 0;
  // Slots into the incoming face. T3 repeats source slot2 in its fourth lane.
  // This changes only the primary; it must never regenerate its SH2 partner.
  unsigned source_corner[4]{0,1,2,3};
  bool reversed = false;
};
} // namespace tlfea::contact::radioss_type25
