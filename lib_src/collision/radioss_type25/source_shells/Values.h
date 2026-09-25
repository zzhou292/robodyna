// SPDX-License-Identifier: AGPL-3.0-or-later
// Ordinary I25STI3/I25INI_GAP_N selection, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Types.h"
#include "../CoefficientUnits.h"
namespace tlfea::contact::radioss_type25::source_shells::detail {
inline unsigned Slots(ShellLayout layout) {
  return layout == ShellLayout::Quad4 ? 4 : (layout == ShellLayout::Triangle3 ? 3 : 0);
}
inline double HalfGap(const PhysicalShell& row, int input_thickness_mode) {
  if (row.part_contact_thickness != 0 && input_thickness_mode == 0)
    return .5 * row.part_contact_thickness;
  if (row.element_thickness != 0 && input_thickness_mode == 0)
    return .5 * row.element_thickness;
  return .5 * row.property_thickness; // Profile excludes stack/property variants.
}
inline bool Supported(const Profile& p) {
  return p.population == Population::OrdinaryShellsOnly && p.property_type == 1 &&
      (p.input_thickness_mode == 0 || p.input_thickness_mode == 1) && p.level == 1 &&
      p.gap_mode == 1 && p.free_edge_gap == 0 && p.contact_thickness_update == 0;
}
} // namespace tlfea::contact::radioss_type25::source_shells::detail
