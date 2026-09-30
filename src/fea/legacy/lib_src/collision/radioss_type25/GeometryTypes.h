// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FrictionTypes.h"
#include <cstddef>
#include <cstdint>
namespace tlfea::contact::radioss_type25 {
enum class GeometryStatus { Ok, InvalidInput, UnsupportedProfile, NonfiniteResult, CapacityExceeded };
struct GeometryProfile {
  int gap_mode = -1, sharp = -1, initial_penetration = -1, damping_flag = -1;
  bool adhesion = false, thermal = false, foreign_row = false;
};
struct StoredNormal { float x = 0, y = 0, z = 0; };
// Identity values are supplied by the source/row owner. This arithmetic packet
// checks consistency, not arbitrary caller authority over a physical owner.
struct GeometryRowKey {
  std::uint64_t secondary_source_id = 0, generation = 0;
  std::size_t history_index = SIZE_MAX;
  int main_segment = 0;
};
template<class U> struct GeometryInput {
  GeometryRowKey key;
  std::uint64_t main_node_ids[4]{};
  Vector main_vertices[4]{}, secondary{};
  // Exact native NOD_NORMAL/EDGE_BISECTOR slot order, not generic nodal
  // normals. T3 slot4 feeds its center; slots3/4 need not be equal.
  StoredNormal corner_normal[4]{};
  int neighbors[4]{};
  int segment_type = 0; // Native MSEGTYP/ETYP, including signed coating tags.
  std::uint64_t boundary_ids[4]{};
  StoredNormal vertex_bisector[4][2]{};
  double main_gap[4]{}, secondary_gap = 0;
  int selection_code = 0; // Actual IRTLM(2); subtriangle=abs(MOD(code,5)).
  double lb = 0, lc = 0; // Current selected native LBM/LCM; no closest-point approximation.
  double incoming_stiffness = 0;
};
// All exposed fields are native-defined after successful evaluation. A finite
// normal is not necessarily unit length: source float32 boundary values persist.
// Keys/generation identify supplied source rows; arithmetic does not mint owner authority.
template<class U> struct RawGeometryResult {
  GeometryRowKey key;
  int selection_code = 0;
  Vector normal{};
  double weights[4]{};
  double geometric_penetration = 0, gap = 0, distance = 0, incoming_stiffness = 0;
};
struct NativeGeometryHistory {
  std::uint64_t secondary_source_id = 0, generation = 0;
  NativeContactRow row;
};
template<class U> struct GeometryFinalResult {
  RawGeometryResult<U> geometry;
  double penetration = 0;
};
using NativeGeometryInput = GeometryInput<NativeUnitsTag>;
using SiGeometryInput = GeometryInput<SiUnitsTag>;
using NativeRawGeometryResult = RawGeometryResult<NativeUnitsTag>;
using SiRawGeometryResult = RawGeometryResult<SiUnitsTag>;
using NativeGeometryFinalResult = GeometryFinalResult<NativeUnitsTag>;
struct GeometryBatchLimits {
  std::size_t rows = 1048576, history_rows = 1048576;
  std::size_t scratch_bytes = std::size_t{1} << 30;
};
struct GeometryBatchForecast {
  GeometryStatus status = GeometryStatus::InvalidInput;
  std::size_t history_scratch_bytes = 0, result_scratch_bytes = 0, total_scratch_bytes = 0;
};
} // namespace tlfea::contact::radioss_type25
