// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "native/LifecycleStages.h"
#include "native/LifecyclePackets.h"
#include "../radioss_type25_selection/NativeOracle.h"
#include "../radioss_type25_local_geometry/NativeOracle.h"
namespace type25_lifecycle_test {
OracleResult OracleLifecycle(const l::Input& input) {
  using namespace reference;
  Tables table(input); ValidateProfileAndSpatial(input);
  OracleResult output; const auto retained = Begin(input,output);
  Rows rows(output.rows.size());
  for (std::size_t i = 0; i < output.rows.size(); ++i) rows.Store(i,output.rows[i].history.row);
  FilterRaw(input,table,rows,output);
  const int row_count = Integer(output.rows.size());
  rd_lifecycle_clear_sliding(&row_count,rows.sliding.data());
  ClassifyRetained(input,retained,rows,output);
  PrepareSliding(input,table,rows,retained,output);
  AppendSliding(input,table,rows,output);
  std::vector<unsigned> initialized(output.occurrences.size(),0);
  for (std::size_t i = 0; i < retained; ++i) initialized[i] = 1; // Original GLOB1 completed.

  // Native COMP_2 materializes the full list before any member mutates a row.
  const auto continuation = Membership(input,table,rows,output,1);
  for (int one_based : continuation) {
    Require(one_based > 0 && std::size_t(one_based) <= output.occurrences.size(), "Native continuation ordinal out of range");
    const auto i = std::size_t(one_based-1); auto& item = output.occurrences[i];
    auto& row = output.rows[item.secondary-1]; ++row.continuation_count;
    const auto native = type25_selection_test::OracleContinuation(input.profile.selection,
        Continuation(input,item,i,row.sliding_reference,row.initial_contact_flag),row.history);
    row.history = native.history; item.cache = native.cache; ++initialized[i];
    rows.Store(std::size_t(item.secondary-1),row.history.row);
  }
  const auto impact = Membership(input,table,rows,output,2);
  for (int one_based : impact) {
    Require(one_based > 0 && std::size_t(one_based) <= output.occurrences.size(), "Native new-impact ordinal out of range");
    const auto i = std::size_t(one_based-1); auto& item = output.occurrences[i];
    auto& row = output.rows[item.secondary-1]; ++row.new_impact_count;
    const auto native = type25_selection_test::OracleNewImpact(input.profile.selection,
        NewImpact(input,item,i,row.initial_contact_flag),row.history);
    row.history = native.history; item.cache = native.cache; ++initialized[i];
    item.local_main = native.cache.local_main; // GLOB22 side rewrite is independent of the global winner.
    rows.Store(std::size_t(item.secondary-1),row.history.row);
  }
  for (unsigned count : initialized)
    Require(count == 1, "Native lifecycle occurrence lacks exactly one completed cache-producing phase");
  Keep(input,table,rows,output);
  output.geometry.resize(output.occurrences.size());
  for (std::size_t i = 0; i < output.occurrences.size(); ++i) {
    auto& item = output.occurrences[i];
    if (item.secondary <= 0) continue;
    const auto geometry = Geometry(input,item,output.rows[item.secondary-1].history);
    // An original undefined-XP read throws here. No production geometry, clamp,
    // invented point or seeded native output enters this composed reference.
    output.geometry[i] = type25_geometry_test::OracleRaw(input.profile.geometry,geometry);
  }
  return output;
}
std::vector<l::RowResult> OracleFinish(const std::vector<l::RowResult>& input) {
  reference::Require(input.size() <= 128, "Native marker-phase row bound exceeded");
  auto output = input; std::vector<int> markers(4*input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    reference::Require(input[i].history.row.irtlm[0] != std::numeric_limits<int>::min(),
        "Native post-force marker negation would overflow");
    for (unsigned k = 0; k < 4; ++k) markers[4*i+k] = input[i].history.row.irtlm[k];
  }
  const int count = reference::Integer(input.size()); rd_lifecycle_finish_markers(&count,markers.data());
  for (std::size_t i = 0; i < input.size(); ++i)
    for (unsigned k = 0; k < 4; ++k) output[i].history.row.irtlm[k] = markers[4*i+k];
  return output;
}
} // namespace type25_lifecycle_test
