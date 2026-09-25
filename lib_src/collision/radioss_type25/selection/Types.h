// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../GeometryTypes.h"
#include <cstddef>
#include <cstdint>
namespace tlfea::contact::radioss_type25::selection {
enum class Status { Ok, InvalidInput, UnsupportedProfile, NonfiniteResult, CapacityExceeded, UndefinedNativeInput };
struct Profile {
  int gap_mode = -1, initial_penetration = -1, local_processor = 0;
  bool foreign_rows = false, thermal = false, gap_loading = false;
};
// Current source-derived point/main-segment operands, before any barycentric
// classification. No final-force K or fabricated selected LB/LC is supplied.
struct NativePairInput {
  GeometryRowKey key;
  std::size_t occurrence = SIZE_MAX;
  int local_main = 0;
  std::uint64_t main_node_ids[4]{};
  Vector main_vertices[4]{}, secondary{};
  StoredNormal normal_slot[4]{};
  int neighbors[4]{}, segment_type = 0;
  std::uint64_t boundary_ids[4]{};
  StoredNormal vertex_bisector[4][2]{};
  double main_gap[4]{}, secondary_gap = 0, main_gap_max = 0;
  // COR3_1/_21/_22 multiply main*abs(secondary) for classification activity.
  // This is distinct from the min/clamped incoming response coefficient.
  double main_coefficient = 0, secondary_coefficient = 0;
  int initial_contact_flag = 0; // Authentic ICONT_I, not an inferred contact.
  double radiation_range = 0, applied_gap = 0;
};
enum SectorChannel : std::uint32_t {
  FarDefined = 1u << 0, PenetrationDefined = 1u << 1,
  DistanceSquaredDefined = 1u << 2, RawBarycentricDefined = 1u << 3,
  ClampedBarycentricDefined = 1u << 4
};
struct SectorValues {
  int far = 0;
  double penetration = 0, distance_squared = 0;
  double raw_lb = 0, raw_lc = 0, clamped_lb = 0, clamped_lc = 0;
  std::uint32_t defined = 0;
};
struct CacheSector {
  int far = 0;
  double penetration = 0, lb = 0, lc = 0;
  // Cached LB/LC are clamped outputs, never silently substituted raw values.
  std::uint32_t defined = 0;
};
struct CandidateCache {
  GeometryRowKey key;
  std::size_t occurrence = SIZE_MAX;
  int local_main = 0;
  CacheSector sector[4]{};
};
struct NativeRetainedResult {
  NativeGeometryHistory history;
  CandidateCache cache;
  SectorValues sector[4]{};
  double classification_product = 0, distance_squared = 0;
  int prior_subtriangle = 0, selected_subtriangle = 0;
  bool active = false;
  // Unassigned channels are deterministic API zeros with mask bits CLEAR.
  // They are not native observations and must never supply consumed barycentrics.
};
} // namespace tlfea::contact::radioss_type25::selection
