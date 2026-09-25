// SPDX-License-Identifier: AGPL-3.0-or-later
#include "LifecycleStages.h"
#include "LifecyclePackets.h"
#include "../../radioss_type25_friction/NativeOracle.h"
#include "../../radioss_type25_selection/NativeOracle.h"
namespace type25_lifecycle_test::reference {
void ValidateProfileAndSpatial(const l::Input& input) {
  const auto& p = input.profile; const auto& source = input.source;
  Require(p.selection.gap_mode == 1 && p.selection.initial_penetration == 5 &&
      p.selection.local_processor == 1 && !p.selection.foreign_rows && !p.selection.thermal &&
      !p.selection.gap_loading && p.geometry.gap_mode == 1 && p.geometry.sharp == 1 &&
      p.geometry.initial_penetration == 5 && p.geometry.damping_flag == 1 &&
      !p.geometry.adhesion && !p.geometry.thermal && !p.geometry.foreign_row &&
      p.coefficient.stiffness_formulation == 4 && p.coefficient.mass_timestep_augmentation == 0 &&
      p.neighbor_removal == 2 && (p.optcd_response_precision == 0 || p.optcd_response_precision == 1 || p.optcd_response_precision == 2), "Unselected lifecycle reference profile");
  Finite(input.step.time); Finite(input.step.previous_dt);
  Require(input.step.time >= 0 && input.step.previous_dt >= 0, "Invalid supplied native time operand");
  Finite(p.minimum_coefficient); Finite(p.maximum_coefficient);
  Require(p.minimum_coefficient >= 0 && p.maximum_coefficient >= p.minimum_coefficient,
      "Invalid native pair stiffness bounds");
  const auto& csr = input.spatial_by_secondary;
  Require(csr.offsets && csr.offset_count == source.secondary_count+1 &&
      csr.entry_count == input.spatial_count && (!csr.entry_count || csr.entries) &&
      csr.offsets[0] == 0 && csr.offsets[source.secondary_count] == csr.entry_count,
      "Spatial occurrence CSR extent mismatch");
  std::vector<bool> seen(input.spatial_count);
  for (std::size_t row = 0; row < source.secondary_count; ++row) {
    Require(csr.offsets[row] <= csr.offsets[row+1] && csr.offsets[row+1] <= csr.entry_count,
        "Spatial occurrence CSR is not monotonic");
    std::size_t previous = 0; bool first = true;
    for (std::size_t j = csr.offsets[row]; j < csr.offsets[row+1]; ++j) {
      const auto ordinal = csr.entries[j];
      Require(ordinal < input.spatial_count && !seen[ordinal] && (first || ordinal > previous),
          "Spatial occurrence CSR changes source order or membership");
      Require(input.spatial[ordinal].secondary == Integer(row+1), "Spatial occurrence belongs to another row");
      seen[ordinal] = true; previous = ordinal; first = false;
    }
  }
  for (bool value : seen) Require(value, "Spatial occurrence CSR omits an input");
  for (std::size_t i = 0; i < input.spatial_count; ++i) {
    const auto& item = input.spatial[i]; const auto pair = Pair(input,item.secondary,item.local_main,i);
    (void)pair; // The raw roster carries identity only; native phases produce its cache.
  }
}
std::size_t Begin(const l::Input& input, OracleResult& out) {
  const auto& source = input.source; out.rows.resize(source.secondary_count);
  for (std::size_t i = 0; i < source.secondary_count; ++i) {
    const auto& prior = input.accepted_rows[i]; auto& row = out.rows[i]; row.history = prior;
    row.initial_contact_flag = source.secondary[i].initial_contact_flag;
    double main_stiffness = 0;
    if (prior.row.irtlm[0] > 0 && source.secondary[i].coefficient != 0 && prior.row.irtlm[3] == 1) {
      const int local = prior.row.irtlm[2];
      Require(local > 0 && std::size_t(local) <= source.main_count, "Prior local main is unavailable");
      Require(source.mains[local-1].global_id == prior.row.irtlm[0], "Prior global/local main mapping disagrees");
      main_stiffness = source.mains[local-1].coefficient;
    }
    const auto begin = type25_friction_test::BeginOracle(prior.row,
        {source.secondary[i].coefficient,main_stiffness,1});
    row.history.row = begin.row;
    if (begin.retained_candidate) {
      l::Occurrence item; item.secondary = Integer(i+1); item.local_main = begin.row.irtlm[2];
      item.origin = l::Origin::Retained; item.source_ordinal = i;
      out.occurrences.push_back(item); row.retained_count = 1;
    }
  }
  return out.occurrences.size();
}
void ClassifyRetained(const l::Input& input, std::size_t retained, Rows& rows, OracleResult& out) {
  for (std::size_t i = 0; i < retained; ++i) {
    auto& item = out.occurrences[i]; auto& row = out.rows[item.secondary-1];
    auto pair = Pair(input,item.secondary,item.local_main,i);
    pair.initial_contact_flag = row.initial_contact_flag;
    const auto native = type25_selection_test::OracleRetained(input.profile.selection,pair,row.history);
    row.history = native.history; item.cache = native.cache;
    rows.Store(std::size_t(item.secondary-1),row.history.row);
  }
}
} // namespace type25_lifecycle_test::reference
