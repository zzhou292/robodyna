// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
namespace type25_geometry_test {
extern "C" void rd_type25_geometry(const double*, const double*, const float*,
    const float*, const int*, double*, int*);
extern "C" void rd_type25_geometry_history(const int*, const int*, const int*,
    const int*, const double*, const double*, const double*, const double*,
    const double*, double*, double*, double*);
namespace {
void RequireProfile(const n::GeometryProfile& p) {
  if (p.gap_mode != 1 || p.sharp != 1 || p.initial_penetration != 5 ||
      p.damping_flag != 1 || p.adhesion || p.thermal || p.foreign_row)
    throw std::invalid_argument("Native geometry oracle requires the selected local profile");
}
void Copy(double* dst, n::Vector v) { dst[0] = v.x; dst[1] = v.y; dst[2] = v.z; }
void Copy(float* dst, n::StoredNormal v) { dst[0] = v.x; dst[1] = v.y; dst[2] = v.z; }
bool Same(n::StoredNormal a, n::StoredNormal b) {
  // A native boundary ID refers to one stored vector. Do not create two bit
  // representations for the same slot, including different signed zeros.
  const float x[]{a.x, a.y, a.z}, y[]{b.x, b.y, b.z};
  return std::memcmp(x, y, sizeof(x)) == 0;
}
bool Same(n::Vector a, n::Vector b) {
  const double x[]{a.x, a.y, a.z}, y[]{b.x, b.y, b.z};
  return std::memcmp(x, y, sizeof(x)) == 0;
}
} // namespace
n::NativeRawGeometryResult OracleRaw(const n::GeometryProfile& p,
                                    const n::NativeGeometryInput& input) {
  RequireProfile(p);
  const int sector = input.selection_code % 5;
  if (!sector || sector > 4 || sector < -4 ||
      (input.main_node_ids[2] == input.main_node_ids[3] && sector != 1 && sector != -1))
    throw std::invalid_argument("Native geometry oracle requires a selected subtriangle");
  double coords[15], scalars[8]{input.lb, input.lc};
  float normals[12], bisectors[24]{};
  int flags[15]{};
  for (unsigned i = 0; i < 4; ++i) {
    if (!input.main_node_ids[i]) throw std::invalid_argument("Zero main node identity");
    Copy(coords + 3*i, input.main_vertices[i]);
    Copy(normals + 3*i, input.corner_normal[i]);
    flags[i] = int(i + 1);
    for (unsigned j = 0; j < i; ++j)
      if (input.main_node_ids[i] == input.main_node_ids[j]) {
        const auto a = input.main_vertices[i], b = input.main_vertices[j];
        if (!Same(a, b))
          throw std::invalid_argument("One native node has conflicting coordinates");
        flags[i] = flags[j];
      }
    flags[4+i] = input.neighbors[i];
    if (input.boundary_ids[i]) {
      unsigned slot = i;
      for (unsigned j = 0; j < i; ++j)
        if (input.boundary_ids[j] == input.boundary_ids[i]) {
          if (!Same(input.vertex_bisector[i][0], input.vertex_bisector[j][0]) ||
              !Same(input.vertex_bisector[i][1], input.vertex_bisector[j][1]))
            throw std::invalid_argument("One native boundary slot has conflicting storage");
          slot = unsigned(flags[8+j] - 1);
          break;
        }
      flags[8+i] = int(slot + 1);
      for (unsigned j = 0; j < 2; ++j)
        Copy(bisectors + 6*slot + 3*j, input.vertex_bisector[i][j]);
    }
    scalars[2+i] = input.main_gap[i];
  }
  Copy(coords + 12, input.secondary);
  scalars[6] = input.secondary_gap; scalars[7] = input.incoming_stiffness;
  flags[12] = input.segment_type; flags[13] = input.selection_code; flags[14] = p.sharp;
  for (double value : coords) if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite coordinate");
  for (double value : scalars) if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite operand");
  for (float value : normals) if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite stored normal");
  for (float value : bisectors) if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite stored bisector");
  double values[11]{}; int defined = 0;
  rd_type25_geometry(coords, scalars, normals, bisectors, flags, values, &defined);
  if (!defined) throw std::invalid_argument("Native branch reads an unassigned closest-point scratch value");
  n::NativeRawGeometryResult result;
  result.key = input.key; result.selection_code = input.selection_code;
  result.normal = {values[0], values[1], values[2]};
  for (unsigned i = 0; i < 4; ++i) result.weights[i] = values[3+i];
  result.geometric_penetration = values[7]; result.gap = values[8];
  result.distance = values[9]; result.incoming_stiffness = values[10];
  return result;
}
HistoryOracleResult OracleHistory(const n::GeometryProfile& p, double time,
    const std::vector<n::NativeRawGeometryResult>& geometry,
    const std::vector<n::NativeGeometryHistory>& selected_rows) {
  RequireProfile(p);
  constexpr std::size_t capacity = 256;
  if (geometry.size() > capacity || selected_rows.size() > capacity ||
      !std::isfinite(time) || time < 0)
    throw std::invalid_argument("Native history oracle bounds exceeded");
  std::array<int, capacity> indices{}, markers{};
  std::array<double, capacity> input_p{}, input_k{}, offsets{}, staged{},
      out_p{}, out_offsets{}, out_staged{};
  for (std::size_t i = 0; i < selected_rows.size(); ++i) {
    markers[i] = selected_rows[i].row.irtlm[0];
    offsets[i] = selected_rows[i].row.penetration_offset;
    staged[i] = selected_rows[i].row.history.normal.staged_stiffness;
    if (!std::isfinite(offsets[i]) || !std::isfinite(staged[i]))
      throw std::invalid_argument("Nonfinite native history input");
  }
  for (std::size_t i = 0; i < geometry.size(); ++i) {
    const auto& g = geometry[i];
    if (g.key.history_index >= selected_rows.size())
      throw std::invalid_argument("Native history row index out of range");
    const auto& row = selected_rows[g.key.history_index];
    const std::int64_t main = row.row.irtlm[0];
    if (g.key.secondary_source_id != row.secondary_source_id || g.key.generation != row.generation ||
        (main < 0 ? -main : main) != g.key.main_segment || g.selection_code != row.row.irtlm[1] ||
        !std::isfinite(g.geometric_penetration) || !std::isfinite(g.incoming_stiffness))
      throw std::invalid_argument("Native history identity or scalar mismatch");
    indices[i] = int(g.key.history_index + 1);
    input_p[i] = g.geometric_penetration; input_k[i] = g.incoming_stiffness;
  }
  const int count = int(geometry.size()), rows = int(selected_rows.size());
  rd_type25_geometry_history(&count, &rows, indices.data(), markers.data(), input_p.data(),
      input_k.data(), offsets.data(), staged.data(), &time, out_p.data(), out_offsets.data(), out_staged.data());
  HistoryOracleResult result;
  result.rows = selected_rows;
  for (std::size_t i = 0; i < selected_rows.size(); ++i) {
    result.rows[i].row.penetration_offset = out_offsets[i];
    result.rows[i].row.history.normal.staged_stiffness = out_staged[i];
  }
  for (std::size_t i = 0; i < geometry.size(); ++i) result.results.push_back({geometry[i], out_p[i]});
  return result;
}
} // namespace type25_geometry_test
