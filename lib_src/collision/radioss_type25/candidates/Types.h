// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../NativeConstants.h"
#include "lib_src/math/HostDevice.h"
#include "lib_src/math/Fixed3Operations.h"
#include <cstddef>
#include <cstdint>
namespace tlfea::contact::radioss_type25::candidates {
using Vector = tl::math::Vec3;
enum class Status { Ok, InvalidInput, UnsupportedProfile, NonfiniteResult,
  ResourceLimit, NotInitialized, AlreadyInitialized, NoReference, StaleReference,
  DeviceFailure, Unusable };
// Arithmetic-only native PEN3 operands after COR3T. Native working units;
// clearance is native length squared, never physical penetration/force.
struct PackedRow {
  std::uint64_t nodes[4]{};
  Vector vertices[4]{}, secondary{};
  double gap = 0, margin = 0;
  int segment_type = 0, main_count = 0, symmetry = 0;
};
struct FilterResult {
  double squared_clearance = 0;
  bool included = false;
};
struct Bounds { Vector minimum{}, maximum{}; };
struct Envelope {Bounds bounds;double radius=0;};
// Exact local TRIVOX pair operands. Exclusions are evaluated from source roster
// identity/removal CSR by the inventory owner before this arithmetic screen.
struct ScreenRow {
  Vector vertices[4]{}, secondary{};
  double margin = 0, curvature = 0, secondary_gap = 0, main_gap = 0;
  double gap_load = 0, drad = 0, stored_motion = 0;
};
// COR3T local row. Five symmetry codes are native ICODT, ordered main1..4,
// secondary; source binding must authenticate every operand. DT1 is the existing
// physical owner's previous-step operand, not an independent timer.
struct LocalRow {
  ScreenRow screen;
  std::uint64_t nodes[4]{}, secondary_node = 0;
  Vector main_velocities[4]{}, secondary_velocity{};
  int constraint_codes[5]{};
  int segment_type = 0, main_count = 0;
  double previous_dt = 0;
};
} // namespace tlfea::contact::radioss_type25::candidates
