// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../TiedSearch.h"
#include "../TiedPatchGeometry.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace tl::constraints::tied_shell::driver_detail {
SearchDriverReport PrepareBounds(const SearchDriverInput& in,
    std::vector<NativeSearchBounds>& bounds, std::vector<double>& radii) {
  bounds.resize(in.master_count);
  radii.resize(in.master_count + in.secondary_count, 0);
  for (std::size_t m = 0; m < in.master_count; ++m) {
    const auto& master = in.masters[m];
    WorkingSearchBoundsInput input;
    input.topology = master.topology;
    input.master_thickness = master.bounds_thickness;
    input.working_length_to_m = in.working_length_to_m;
    for (unsigned n = 0; n < 4; ++n)
      input.master_position[n] = in.working_positions[master.nodes[n]];
    const auto status = PrepareSearchBounds(input, bounds[m]);
    if (status != Status::Success)
      return Error(SearchDriverStatus::NumericalFailure, "Tied native bounds rejected", SIZE_MAX, m, status);
    radii[m] = std::nextafter(bounds[m].inflation, std::numeric_limits<double>::infinity());
    if (!std::isfinite(radii[m]))
      return Error(SearchDriverStatus::NumericalFailure, "Tied outward radius overflows", SIZE_MAX, m);
  }
  return {};
}
namespace {
WorkingSearchInput Packet(const SearchDriverInput& in, std::size_t secondary, std::size_t master) {
  const auto& row = in.masters[master];
  WorkingSearchInput out;
  out.topology = row.topology;
  out.master_thickness = row.projection_thickness;
  out.working_length_to_m = in.working_length_to_m;
  out.geometry.secondary_position = in.working_positions[in.secondary_nodes[secondary]];
  for (unsigned n = 0; n < 4; ++n)
    out.geometry.master_position[n] = in.working_positions[row.nodes[n]];
  return out;
}
}
SearchDriverReport Reduce(const SearchDriverInput& in, std::vector<CollisionPair>& pairs,
    const std::vector<NativeSearchBounds>& bounds, SearchDriverResult& staged) {
  const auto primitives = in.master_count + in.secondary_count;
  for (auto& pair : pairs) {
    const auto a = std::min(pair.idA, pair.idB);
    const auto b = std::max(pair.idA, pair.idB);
    if (a < 0 || std::size_t(a) >= in.master_count || std::size_t(b) < in.master_count ||
        std::size_t(b) >= primitives)
      return Error(SearchDriverStatus::InvalidInput, "Invalid cross-mesh tied candidate");
    pair.idA = a;
    pair.idB = b - in.master_count;
  }
  std::sort(pairs.begin(), pairs.end(), [](const CollisionPair& a, const CollisionPair& b) {
    return a.idB < b.idB || (a.idB == b.idB && a.idA < b.idA);
  });
  staged.rows.resize(in.secondary_count);
  for (std::size_t p = 0; p < pairs.size(); ++p) {
    const auto m = std::size_t(pairs[p].idA);
    const auto s = std::size_t(pairs[p].idB);
    if (p && pairs[p-1].idA == pairs[p].idA && pairs[p-1].idB == pairs[p].idB)
      return Error(SearchDriverStatus::InvalidInput, "Repeated tied candidate pair", s, m);
    auto& result = staged.rows[s];
    ++result.candidates;
    const auto& nodes = in.masters[m].nodes;
    if (std::find(nodes.begin(), nodes.end(), in.secondary_nodes[s]) != nodes.end()) {
      ++result.excluded_own_node;
      continue;
    }
    bool within = false;
    const auto box = WithinWorkingSearchBounds(bounds[m], in.working_positions[in.secondary_nodes[s]], within);
    if (box != Status::Success)
      return Error(SearchDriverStatus::NumericalFailure, "Tied native box rejected", s, m, box);
    if (!within) continue;
    ++result.within_native_bounds;
    const auto input = Packet(in, s, m);
    CandidateProjection projection;
    const auto status = ProjectCandidate(input, projection);
    if (status != Status::Success)
      return Error(SearchDriverStatus::NumericalFailure, "Tied candidate projection rejected", s, m, status);
    result.admissible_candidates += projection.admissible;
    // Keep selection in the existing qualified value entry. The independent
    // projection above supplies diagnostic counts; no comparison is copied.
    const auto considered = ConsiderCandidate(input, m + 1, result.choice);
    if (considered != Status::Success)
      return Error(SearchDriverStatus::NumericalFailure, "Tied ordered selection rejected", s, m, considered);
  }
  for (std::size_t s = 0; s < staged.rows.size(); ++s) {
    auto& result = staged.rows[s];
    if (!result.choice.matched) continue;
    ++staged.matched_count;
    auto input = Packet(in, s, result.choice.ordered_master - 1).geometry;
    input.secondary_position = tl::math::fixed3::Scale(input.secondary_position, in.working_length_to_m);
    for (auto& x : input.master_position)
      x = tl::math::fixed3::Scale(x, in.working_length_to_m);
    result.force_patch_status = PreparePatch(input, result.force_patch);
    staged.singular_patch_count += result.force_patch_status == Status::SingularPatch;
  }
  staged.pair_count = pairs.size();
  return {};
}
} // namespace tl::constraints::tied_shell::driver_detail
