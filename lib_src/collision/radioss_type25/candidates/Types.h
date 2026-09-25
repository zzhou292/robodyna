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
// Exact local TRIVOX pair operands. Exclusions are evaluated from source roster
// identity/removal CSR by the inventory owner before this arithmetic screen.
struct ScreenRow {
  Vector vertices[4]{}, secondary{};
  double margin = 0, curvature = 0, secondary_gap = 0, main_gap = 0;
  double gap_load = 0, drad = 0, stored_motion = 0;
};
} // namespace tlfea::contact::radioss_type25::candidates
