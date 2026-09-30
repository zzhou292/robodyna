// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <initializer_list>
namespace type25_selection_test {
extern "C" void rd_selection_retained(const double*, const double*, const float*, const float*,
    const int*, const double*, double*, int*, int*, double*, int*);
namespace {
void Pack(double* out, n::Vector value) { out[0] = value.x; out[1] = value.y; out[2] = value.z; }
void Pack(float* out, n::StoredNormal value) { out[0] = value.x; out[1] = value.y; out[2] = value.z; }
bool Same(n::StoredNormal a, n::StoredNormal b) {
  const float x[]{a.x, a.y, a.z}, y[]{b.x, b.y, b.z};
  return std::memcmp(x, y, sizeof(x)) == 0;
}
bool Same(n::Vector a, n::Vector b) {
  const double x[]{a.x, a.y, a.z}, y[]{b.x, b.y, b.z};
  return std::memcmp(x, y, sizeof(x)) == 0;
}
void RequireFinite(double value) {
  if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite native retained oracle operand/output");
}
} // namespace
s::NativeRetainedResult OracleRetained(const s::Profile& profile, const s::NativePairInput& input,
    const n::NativeGeometryHistory& prior, double seed, RetainedScratchObservation* observation) {
  if (profile.gap_mode != 1 || profile.initial_penetration != 5 || profile.local_processor != 1 ||
      profile.foreign_rows || profile.thermal || profile.gap_loading ||
      input.radiation_range != 0 || input.applied_gap != 0)
    throw std::invalid_argument("Unselected retained oracle profile");
  if (!input.key.secondary_source_id || input.key.secondary_source_id != prior.secondary_source_id ||
      input.key.generation != prior.generation || input.local_main <= 0 || input.key.main_segment <= 0)
    throw std::invalid_argument("Inconsistent retained source/row identity");
  RequireFinite(seed);
  double coordinates[15], scalars[12]{input.main_coefficient, input.secondary_coefficient,
      input.secondary_gap, input.main_gap_max};
  float normals[12], bisectors[24]{}; int flags[19]{};
  for (unsigned i = 0; i < 4; ++i) {
    if (!input.main_node_ids[i]) throw std::invalid_argument("Zero source node identity");
    Pack(coordinates + 3*i, input.main_vertices[i]); Pack(normals + 3*i, input.normal_slot[i]);
    flags[i] = int(i + 1);
    for (unsigned j = 0; j < i; ++j) if (input.main_node_ids[i] == input.main_node_ids[j]) {
      const auto a = input.main_vertices[i], b = input.main_vertices[j];
      if (!Same(a, b))
        throw std::invalid_argument("One native node has conflicting current positions");
      flags[i] = flags[j];
    }
    flags[4+i] = input.neighbors[i];
    if (input.boundary_ids[i]) {
      unsigned slot = i;
      for (unsigned j = 0; j < i; ++j) if (input.boundary_ids[i] == input.boundary_ids[j]) {
        if (!Same(input.vertex_bisector[i][0], input.vertex_bisector[j][0]) ||
            !Same(input.vertex_bisector[i][1], input.vertex_bisector[j][1]))
          throw std::invalid_argument("One boundary reference has conflicting native float storage");
        slot = unsigned(flags[8+j] - 1); break;
      }
      flags[8+i] = int(slot + 1);
      for (unsigned j = 0; j < 2; ++j) Pack(bisectors + 6*slot + 3*j, input.vertex_bisector[i][j]);
    }
    scalars[4+i] = input.main_gap[i];
    flags[14+i] = prior.row.irtlm[i];
  }
  Pack(coordinates + 12, input.secondary);
  scalars[8] = input.radiation_range; scalars[9] = input.applied_gap;
  scalars[10] = prior.row.selection_metric[0]; scalars[11] = prior.row.selection_metric[1];
  flags[12] = input.segment_type; flags[13] = input.key.main_segment;
  flags[18] = input.initial_contact_flag;
  for (double value : coordinates) RequireFinite(value);
  for (double value : scalars) RequireFinite(value);
  for (float value : normals) RequireFinite(value);
  for (float value : bisectors) RequireFinite(value);
  const int signed_prior = prior.row.irtlm[1] % 5;
  const int prior_sector = signed_prior < 0 ? -signed_prior : signed_prior;
  double values[36], row_values[4]; int sector_flags[16], markers[4], defined = 0;
  rd_selection_retained(coordinates, scalars, normals, bisectors, flags, &seed,
      values, sector_flags, markers, row_values, &defined);
  // The wrapper observes the ACTUAL native classification product before the
  // subscript check, including underflow-to-zero inactive products.
  if (!defined) throw std::invalid_argument("Native retained phase has an undefined old-sector input");
  s::NativeRetainedResult result;
  result.history = prior;
  for (unsigned i = 0; i < 4; ++i) result.history.row.irtlm[i] = markers[i];
  result.history.row.selection_metric[0] = row_values[2];
  result.history.row.selection_metric[1] = row_values[3];
  result.classification_product = row_values[0]; result.distance_squared = row_values[1];
  for (double value : row_values) RequireFinite(value);
  result.active = row_values[0] > 0; result.prior_subtriangle = prior_sector;
  const int selected = markers[1] % 5;
  result.selected_subtriangle = result.active ? (selected < 0 ? -selected : selected) : 0;
  result.cache.key = input.key; result.cache.occurrence = input.occurrence;
  result.cache.local_main = input.local_main;
  for (unsigned i = 0; i < 4; ++i) {
    const auto* value = values + 9*i; const auto* flag = sector_flags + 4*i;
    auto& sector = result.sector[i]; auto& cache = result.cache.sector[i];
    sector.far = flag[0]; sector.penetration = value[4]; sector.distance_squared = value[8];
    sector.defined = s::FarDefined | s::PenetrationDefined | s::DistanceSquaredDefined;
    cache.far = flag[1]; cache.penetration = value[5];
    cache.defined = s::FarDefined | s::PenetrationDefined;
    RequireFinite(value[4]); RequireFinite(value[5]); RequireFinite(value[8]);
    if (flag[2]) {
      RequireFinite(value[0]); RequireFinite(value[1]);
      sector.raw_lb = value[0]; sector.raw_lc = value[1]; sector.defined |= s::RawBarycentricDefined;
    }
    if (flag[3]) {
      for (unsigned j : {2u, 3u, 6u, 7u}) RequireFinite(value[j]);
      sector.clamped_lb = value[2]; sector.clamped_lc = value[3];
      sector.defined |= s::ClampedBarycentricDefined;
      cache.lb = value[6]; cache.lc = value[7]; cache.defined |= s::ClampedBarycentricDefined;
    }
    if (observation) {
      observation->raw_lb[i] = value[0]; observation->raw_lc[i] = value[1];
      observation->clamped_lb[i] = value[2]; observation->clamped_lc[i] = value[3];
      observation->cache_lb[i] = value[6]; observation->cache_lc[i] = value[7];
    }
  }
  return result;
}
} // namespace type25_selection_test
