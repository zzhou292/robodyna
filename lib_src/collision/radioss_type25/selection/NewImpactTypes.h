// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
namespace tlfea::contact::radioss_type25::selection {
struct NativeOppositeSide {
  int local_main = 0, global_main = 0;
  std::uint64_t main_node_ids[4]{};
  StoredNormal normal_slot[4]{};
  int neighbors[4]{};
  std::uint64_t boundary_ids[4]{};
  StoredNormal vertex_bisector[4][2]{};
  // These are authentic partner operands. The scalar COR22 arithmetic does
  // not manufacture or prove the main/partner topology correspondence; that
  // correspondence is an explicit source-binding/coordinator admission gate.
};
struct NativeNewImpactInput {
  NativePairInput pair;
  int segment_count = 0; // Actual NRTM; pair.segment_type is native MSEGTYP.
  NativeOppositeSide opposite;
  Vector main_velocity[4]{}, secondary_velocity{};
  double previous_dt = 0; // Native DT1, never the proposed next-step DT2.
  // COR22 keeps the PRIMARY gaps and classification product if side B wins.
  // Incoming response stiffness is resolved separately on the selected face.
};
enum class ImpactSide { None, Primary, Opposite };
enum ImpactScalarChannel : std::uint32_t {
  ImpactPenetrationDefined = 1u << 0,
  ImpactWeightsAndFarDefined = 1u << 1
};
struct ImpactSideValues {
  int far[4]{}, cylindrical_gap[4]{};
  double penetration[4]{};
  int subtriangle = 0, intersection = 0;
  // All per-sector arrays are native-initialized. The selector denotes the
  // side's candidate before the strict side-A versus side-B winner decision.
};
struct NativeNewImpactResult {
  NativeGeometryHistory history;
  CandidateCache cache;
  GeometryRowKey source_key;
  int source_local_main = 0;
  SectorValues projection[4]{};
  ImpactSideValues primary, opposite;
  double classification_product = 0;
  double penetration = 0, lb = 0, lc = 0;
  int far = 0, selected_subtriangle = 0, recontact_intersection = 0;
  ImpactSide selected_side = ImpactSide::None;
  std::uint32_t scalar_defined = 0;
  bool active = false, row_replaced = false;
  // Inactive PENT and non-winning FAR/LBS/LCS are masked API zeros.
  // GLOB22 clears all four cache sectors even for inactive/equal-side rows:
  // those cache zeros are native-defined. A local side-B winner changes the
  // occurrence's cached main even when the GLOBAL row winner is unchanged.
};
} // namespace tlfea::contact::radioss_type25::selection
